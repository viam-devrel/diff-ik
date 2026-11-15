// Phase 2 TypeScript smoke test: the same command sequence as the Python bench's smoke step.
// Plaintext direct dial (HOST=http://localhost:<port>) works in the SDK only if the host contains
// "local" (dialDirect rejects e.g. 127.0.0.1), so use localhost. Otherwise set VIAM_API_KEY_ID/VIAM_API_KEY
// to use WebRTC signaling via the node-datachannel polyfill.
import * as VIAM from '@viamrobotics/sdk';
import { createGrpcTransport } from '@connectrpc/connect-node';

(globalThis as any).VIAM = {
  GRPC_TRANSPORT_FACTORY: (opts: any) => createGrpcTransport({ httpVersion: '2', ...opts }),
};

const env = (k: string, d?: string): string => {
  const v = process.env[k] ?? d;
  if (v === undefined) throw new Error(`set ${k}`);
  return v;
};
const arm = env('ARM');
const fails: string[] = [];
const check = (name: string, ok: boolean, detail: unknown) => {
  console.log(`${name.padEnd(30)} ${ok ? 'PASS' : 'FAIL'}  ${JSON.stringify(detail)}`);
  if (!ok) fails.push(name);
};
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

async function main() {
  const keyId = process.env.VIAM_API_KEY_ID;
  const key = process.env.VIAM_API_KEY;
  if (keyId && key) {
    // Cloud signaling dials WebRTC; Node has no RTCPeerConnection, so install the polyfill
    // (as ~/src/viam-typescript-sdk/examples/node/src/main.ts does).
    const wrtc = await import('node-datachannel/polyfill');
    for (const k in wrtc) (globalThis as any)[k] = (wrtc as any)[k];
  }
  // createRobotClient reports success only for a WebRTC peer connection (isConnected checks ICE
  // state), so a working direct gRPC dial would throw "Failed to connect". Dial directly instead.
  const client =
    keyId && key
      ? await VIAM.createRobotClient({
          host: env('HOST'),
          noReconnect: true,
          credentials: { type: 'api-key', authEntity: keyId, payload: key },
          signalingAddress: 'https://app.viam.com:443',
          iceServers: [{ urls: 'stun:global.stun.twilio.com:3478' }],
        } as any)
      : await new VIAM.RobotClient().dial({ host: env('HOST'), noReconnect: true } as any);
  const svc = new VIAM.MotionClient(client, env('SERVICE', 'servo'));
  const call = (c: Record<string, any>) => svc.doCommand(c) as Promise<any>;
  const expectErr = async (name: string, sub: string, c: Record<string, any>) => {
    try {
      await call(c);
      check(name, false, 'no error');
    } catch (e) {
      check(name, String(e).includes(sub), String(e).slice(0, 80));
    }
  };
  const start = (mode: string) => ({
    command: 'servo_start', component_name: arm, frame: `${arm}_origin`, mode,
    limits: { joint_deg_s: Number(env('JOINT_DEG_S', '90')) },
  });

  try {
    const r = await call(start('pose'));
    const sid = r.session_id as string;
    const goal = { ...r.pose, z: r.pose.z + 10 };
    await expectErr('busy arm', 'busy', start('pose'));
    await expectErr('wrong mode', 'wrong mode', { command: 'servo_twist', session_id: sid, linear: [0, 0, 0], angular: [0, 0, 0] });
    await expectErr('unknown session', 'unknown session_id', { command: 'servo_status', session_id: 'nope' });
    await expectErr('unknown command', 'unknown command', { command: 'servo_fly' });
    let state = '';
    for (let i = 0; i < 60 && state !== 'converged'; i++) {
      state = (await call({ command: 'servo_target', session_id: sid, pose: goal })).state;
      await sleep(100);
    }
    check('pose converges', state === 'converged', state);
    const st = await call({ command: 'servo_status', session_id: sid });
    check('status has fields', typeof st.rate_hz === 'number' && typeof st.limits === 'object', Object.keys(st));
    check('pose stop', (await call({ command: 'servo_stop', session_id: sid })).state === 'stopped', 'stop');
    await expectErr('ended session', 'session ended', { command: 'servo_target', session_id: sid, pose: goal });

    const tw = (await call(start('twist'))).session_id as string;
    for (let i = 0; i < 5; i++) {
      await call({ command: 'servo_twist', session_id: tw, linear: [0, 0, -10], angular: [0, 0, 0] });
      await sleep(100);
    }
    check('twist stop', (await call({ command: 'servo_stop', session_id: tw })).state === 'stopped', 'stop');
  } finally {
    await client.disconnect();
  }
  console.log(fails.length ? `FAILED: ${fails.join(', ')}` : 'ALL PASS');
  process.exit(fails.length ? 1 : 0);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
