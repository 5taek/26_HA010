"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");
const vm = require("node:vm");

// Exercise the actual HTTP handler without accessing Firebase or real secrets.
function loadHandler(secret, options = {}) {
  let handler;
  const calls = {reads: 0, commits: 0};
  const stored = new Map();
  let notifications = 0;
  let generatedIds = 0;
  let transactionTail = Promise.resolve();
  let failCommit = options.failCommitOnce;
  function collection(collectionPath) {
    return {
      doc(id = `generated-${++generatedIds}`) {
        const documentPath = `${collectionPath}/${id}`;
        return {
          id, path: documentPath,
          collection: (name) => collection(`${documentPath}/${name}`),
          async get() {
            calls.reads++;
            const data = collectionPath === "devices" ? {
              patientId: "test-patient", guardianUids: ["test-guardian"],
              ...stored.get(documentPath),
            } : stored.get(documentPath);
            return {exists: data !== undefined, data: () => data,
              get: (key) => data?.[key]};
          },
          async set(value) {
            stored.set(documentPath, {...stored.get(documentPath), ...value});
          },
        };
      },
      async get() {
        return {forEach: (callback) => callback({get: () => "test-fcm-token"})};
      },
    };
  }
  const db = {
    collection,
    async runTransaction(callback) {
      const previous = transactionTail;
      let release;
      transactionTail = new Promise((resolve) => {release = resolve;});
      await previous;
      try {
        const writes = [];
        const result = await callback({get: (ref) => ref.get(),
          set: (ref, value) => writes.push([ref.path, value])});
        if (failCommit) {failCommit = false; throw new Error("test storage failure");}
        if (writes.length) {
          for (const [key, value] of writes) stored.set(key, {...stored.get(key), ...value});
          calls.commits++;
        }
        return result;
      } finally {release();}
    },
  };
  const dependencies = {
    "firebase-functions/v2/https": {
      onRequest: (_, callback) => {handler = callback; return callback;},
    },
    "firebase-functions/params": {defineSecret: () => ({value: () => secret})},
    "firebase-admin/app": {initializeApp() {}},
    "firebase-admin/firestore": {
      getFirestore: () => db, FieldValue: {serverTimestamp: () => 0},
    },
    "firebase-admin/messaging": {
      getMessaging: () => ({async sendEachForMulticast() {
        notifications++;
        return {successCount: 1, responses: [{success: true}]};
      }}),
    },
  };
  vm.runInNewContext(
      fs.readFileSync(path.join(__dirname, "index.js"), "utf8"),
      {exports: {}, Buffer, console, setTimeout,
        require: (name) => dependencies[name] ?? require(name)},
  );
  return {
    calls,
    stored,
    notifications: () => notifications,
    async request(deviceId, token, payload = {}) {
      const response = {
        status(code) {this.code = code; return this;},
        json(body) {this.body = body; return this;},
      };
      await handler({method: "POST", body: {deviceId, state: "safe_fall", ...payload},
        get: () => token}, response);
      return response;
    },
  };
}

test("registered device with exact token can write an event", async () => {
  const server = loadHandler('{"cane-001":"test-token"}');
  const response = await server.request("cane-001", "test-token");
  assert.equal(response.code, 201);
  assert.equal(server.calls.commits, 1);
});

for (const [name, deviceId, token] of [
  ["missing token", "cane-001", undefined],
  ["empty token", "cane-001", ""],
  ["whitespace token", "cane-001", "   "],
  ["wrong token", "cane-001", "wrong-token"],
  ["unregistered device without token", "unknown-device", undefined],
  ["unregistered device with another device token", "unknown-device", "test-token"],
  ["inherited property as device ID", "toString", "test-token"],
]) {
  test(`${name} is rejected before Firestore access`, async () => {
    const server = loadHandler('{"cane-001":"test-token"}');
    assert.equal((await server.request(deviceId, token)).code, 401);
    assert.deepEqual(server.calls, {reads: 0, commits: 0});
  });
}

for (const token of ["", "   ", null, 123]) {
  test(`invalid configured token ${JSON.stringify(token)} is rejected`, async () => {
    const server = loadHandler(JSON.stringify({"cane-001": token}));
    assert.equal((await server.request("cane-001", "test-token")).code, 401);
    assert.deepEqual(server.calls, {reads: 0, commits: 0});
  });
}

for (const secret of ["null", "[]", '"text"', "123", "invalid-json"]) {
  test(`invalid secret shape ${secret} returns a configuration error`, async () => {
    const server = loadHandler(secret);
    assert.equal((await server.request("cane-001", "test-token")).code, 500);
    assert.deepEqual(server.calls, {reads: 0, commits: 0});
  });
}

test("lost HTTP response can be retried without duplicate records or pushes", async () => {
  const server = loadHandler('{"cane-001":"test-token"}');
  const payload = {eventId: "fall-001", state: "danger_fall", deviceTimeMs: 123};
  const first = await server.request("cane-001", "test-token", payload);
  const retry = await server.request("cane-001", "test-token", payload);
  assert.equal(first.code, 201);
  assert.equal(retry.code, 200);
  assert.equal(retry.body.duplicate, true);
  assert.equal(retry.body.eventId, first.body.eventId);
  assert.equal(server.calls.commits, 1);
  assert.equal(server.notifications(), 1);
});

test("concurrent requests for one event commit once", async () => {
  const server = loadHandler('{"cane-001":"test-token"}');
  const responses = await Promise.all(Array.from({length: 8}, () =>
    server.request("cane-001", "test-token", {eventId: "same", state: "danger_fall"})));
  assert.equal(responses.filter((response) => response.code === 201).length, 1);
  assert.equal(responses.filter((response) => response.code === 200).length, 7);
  assert.equal(server.calls.commits, 1);
  assert.equal(server.notifications(), 1);
});

test("failed storage commit is retryable with the original event ID", async () => {
  const server = loadHandler('{"cane-001":"test-token"}', {failCommitOnce: true});
  const payload = {eventId: "retry-storage"};
  assert.equal((await server.request("cane-001", "test-token", payload)).code, 500);
  assert.equal((await server.request("cane-001", "test-token", payload)).code, 201);
  assert.equal(server.calls.commits, 1);
});

test("same event ID on different devices represents different events", async () => {
  const server = loadHandler('{"cane-001":"one","cane-002":"two"}');
  const first = await server.request("cane-001", "one", {eventId: "same"});
  const second = await server.request("cane-002", "two", {eventId: "same"});
  assert.equal(first.code, 201);
  assert.equal(second.code, 201);
  assert.notEqual(first.body.eventId, second.body.eventId);
  assert.equal(server.calls.commits, 2);
});

test("reusing an event ID with changed contents is rejected", async () => {
  const server = loadHandler('{"cane-001":"test-token"}');
  await server.request("cane-001", "test-token", {eventId: "fixed"});
  assert.equal((await server.request("cane-001", "test-token",
      {eventId: "fixed", state: "danger_fall"})).code, 409);
  assert.equal(server.calls.commits, 1);
  assert.equal(server.notifications(), 0);
});

test("invalid event IDs are rejected", async () => {
  const server = loadHandler('{"cane-001":"test-token"}');
  for (const eventId of ["", "a/b", null, 123, "x".repeat(129)]) {
    assert.equal((await server.request("cane-001", "test-token", {eventId})).code, 400);
  }
  assert.deepEqual(server.calls, {reads: 0, commits: 0});
});

test("delayed older safe event cannot replace the newer danger status", async () => {
  const server = loadHandler('{"cane-001":"test-token"}');
  await server.request("cane-001", "test-token",
      {eventId: "newer", eventSequence: 2, state: "danger_fall"});
  await server.request("cane-001", "test-token",
      {eventId: "older", eventSequence: 1, state: "safe_fall"});
  assert.equal(server.calls.commits, 2);
  assert.equal(server.stored.get("devices/cane-001").latestState, "danger_fall");
  assert.equal(server.stored.get("devices/cane-001").latestEventSequence, 2);
});
