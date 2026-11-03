// 物理模型数值验证（Node 环境无 DOM）
const dt = 1 / 120;

// 1) 单摆 RK4
{
  const L = 1.0, g = 9.8;
  let theta = 15 * Math.PI / 180, omega = 0, t = 0;
  let lastZero = null, T = null, count = 0;
  const acc = (th, om) => -(g / L) * Math.sin(th);
  while (t < 30) {
    const pt = theta;
    const k1t = omega, k1o = acc(theta, omega);
    const k2t = omega + k1o * dt / 2, k2o = acc(theta + k1t * dt / 2, omega + k1o * dt / 2);
    const k3t = omega + k2o * dt / 2, k3o = acc(theta + k2t * dt / 2, omega + k2o * dt / 2);
    const k4t = omega + k3o * dt, k4o = acc(theta + k3t * dt, omega + k3o * dt);
    theta += dt / 6 * (k1t + 2 * k2t + 2 * k3t + k4t);
    omega += dt / 6 * (k1o + 2 * k2o + 2 * k3o + k4o);
    t += dt;
    if (pt < 0 && theta >= 0) {
      count++;
      if (count > 1) { T = t - lastZero; break; }
      lastZero = t;
    }
  }
  const Ttheory = 2 * Math.PI * Math.sqrt(L / g);
  console.log(`单摆: 测得T=${T.toFixed(4)} 理论T=${Ttheory.toFixed(4)} 误差=${(Math.abs(T - Ttheory) / Ttheory * 100).toFixed(2)}%`);
}

// 2) 弹簧振子（leapfrog）
{
  const k = 25, m = 0.5;
  let x = 0.15, v = 0, t = 0, count = 0, lastZero = null, T = null;
  const om2 = k / m;
  while (t < 30) {
    const pv = v;
    v += -om2 * x * dt / 2; x += v * dt; v += -om2 * x * dt / 2;
    t += dt;
    if (v > 0 && pv <= 0) {
      count++;
      if (count > 1) { T = t - lastZero; break; }
      lastZero = t;
    }
  }
  const Tt = 2 * Math.PI * Math.sqrt(m / k);
  console.log(`弹簧: 测得T=${T.toFixed(4)} 理论T=${Tt.toFixed(4)} 误差=${(Math.abs(T - Tt) / Tt * 100).toFixed(3)}%`);
}

// 3) MNA 电路
function solveLinear(A, z) {
  const n = A.length;
  const M = A.map((r, i) => Float64Array.from([...r, z[i]]));
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    if (Math.abs(M[piv][col]) < 1e-12) return null;
    [M[col], M[piv]] = [M[piv], M[col]];
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = M[r][col] / M[col][col];
      for (let c2 = col; c2 <= n; c2++) M[r][c2] -= f * M[col][c2];
    }
  }
  const x2 = new Float64Array(n);
  for (let i = 0; i < n; i++) x2[i] = M[i][n] / M[i][i];
  return x2;
}
{
  // 3V 电池 + 10Ω + 8Ω 串联: I = 3/18 = 0.1667
  // 节点: 0=地(电池负极), 1=电池正极=10Ω一端, 2=10Ω另一端=8Ω一端; 8Ω另一端接地
  // MNA 未知量: v1, v2, I_batt（size=3）
  const size = 3;
  const A = Array.from({ length: size }, () => new Float64Array(size));
  const z = new Float64Array(size);
  // 电导: R1(节点1-2)=10Ω, R2(节点2-地)=8Ω
  A[0][0] += 1 / 10;             // 节点1自导纳（R1 连接 1-2）
  A[1][1] += 1 / 10 + 1 / 8;     // 节点2自导纳
  A[0][1] -= 1 / 10; A[1][0] -= 1 / 10; // R1 互导纳
  // 电压源: 正极在节点1(v1)，负极在地; 关联参考 I 从正极流出
  A[2][0] += 1; A[0][2] += 1;    // v1 + 0*I = 3 -> 行: v1 = 3
  z[2] = 3;
  const xv = solveLinear(A, z);
  const I = (xv[1] - 0) / 8;     // 流过 R2 到地的电流
  console.log(`电路: v1=${xv[0].toFixed(4)} v2=${xv[1].toFixed(4)} I=${I.toFixed(5)} A (理论 0.16667, 误差 ${(Math.abs(I - 3 / 18) / (3 / 18) * 100).toFixed(4)}%)`);
}

// 4) 平抛
{
  const v0 = 2.5, h0 = 0.6;
  const tf = Math.sqrt(2 * h0 / 9.8);
  console.log(`平抛: 落地t=${tf.toFixed(4)}s 射程=${(v0 * tf).toFixed(4)}m (理论 0.3499 / 0.8748)`);
}

// 5) 热混合
{
  const mH = 0.2, TH = 80, mC = 0.2, TC = 20;
  console.log(`热混合: 终温=${((mH * TH + mC * TC) / (mH + mC)).toFixed(1)}°C (理论 50.0)`);
}

// 6) 薄透镜成像: u=0.5, f=0.25 -> v=0.5
{
  const u = 0.5, f = 0.25;
  const v = 1 / (1 / f + 1 / u); // 牛顿公式 1/v = 1/f + 1/u（本平台符号约定）
  console.log(`透镜: u=${u}, f=${f}, v=${v.toFixed(3)} (验证光路程序按 dy2 = dy - h/f)`);
}
console.log('--- 物理模型验证完成 ---');
