// Mockup of the monitor for trying the web page without one (docs section 8.7).
// It answers the page's /api/... requests in the browser with made-up values from a small
// room simulation: two people, sleeping, cooking, airing, heating, the weather.
// make_web_page.ps1 puts it into docs/mockup/index.html, in front of the page's own script.
// The answers have the same form as the firmware's (web.cpp, history.cpp).
(function () {
  'use strict';
  const DAY = 86400, MIN = 60;
  const HISTORY_DAYS = 60;            // made-up history: 1-minute values for the last 14 days, 5-minute before
  const KEEP = {m1: 14, m5: 1096};    // keepDays as in the firmware
  const LIMITS = [600, 1000, 1500], VOC_LIMITS = [150, 250, 400];
  const MAX_SPAN = {1: 2 * DAY, 5: 14 * DAY, days: 45 * DAY};

  const opened = Date.now() / 1000;
  const utcDay = t => t - t % DAY;
  const S0 = utcDay(opened) - HISTORY_DAYS * DAY;   // first minute of the history
  const oldest = {m1: utcDay(opened) - KEEP.m1 * DAY, m5: S0};
  const lastMinute = () => Math.floor(Date.now() / 1000 / MIN) * MIN - MIN;  // newest finished minute

  // Repeatable random numbers 0..1 and smooth noise over time
  const hash = n => {
    n = Math.imul((n | 0) ^ 0x9e3779b9, 0x85ebca6b);
    n = Math.imul(n ^ (n >>> 13), 0xc2b2ae35);
    return ((n ^ (n >>> 16)) >>> 0) / 4294967296;
  };
  const rnd = (a, b) => hash(a * 7919 + b * 104729);
  const noise = (t, period, seed) => {
    const x = t / period, i = Math.floor(x), f = x - i, s = f * f * (3 - 2 * f);
    return rnd(i, seed) * (1 - s) + rnd(i + 1, seed) * s;
  };

  // Local midnight a number of days before today, for the gaps below
  const localDay = (back, h, m) => { const d = new Date(); return new Date(d.getFullYear(), d.getMonth(), d.getDate() - back, h, m).getTime() / 1000; };
  const GAPS = [
    {from: localDay(9, 14, 0), to: localDay(9, 15, 35), learn: true},    // power off 95 min: the VOC index learns again
    {from: localDay(3, 10, 12), to: localDay(3, 10, 16), learn: false},  // restart (upload): VOC state restored
  ];
  const restartAt = GAPS[1].to;

  // The day plan of the room, different every day
  function plan(d) {
    const key = d.getFullYear() * 400 + d.getMonth() * 31 + d.getDate(), r = k => rnd(key, k);
    const weekend = d.getDay() === 0 || d.getDay() === 6;
    return {
      wake: weekend ? 8 + r(1) : 6.4 + 0.5 * r(1),
      leave: weekend ? (r(2) > 0.5 ? 13 + r(3) : 99) : 7.8 + 0.3 * r(2),
      back: weekend ? 15.5 + r(4) : 16.4 + 1.5 * r(4),
      bed: 22.4 + 1.1 * r(5),
      sleepers: r(6) > 0.85 ? 3 : 2,                         // a guest now and then: worse air at night
      airEvening: r(7) > 0.35 ? 19.6 + 2 * r(8) : -1,
      airMinutes: 5 + 12 * r(9),
      cook: r(10) > 0.15 ? 18.2 + r(11) : -1,
      cookVoc: 90 + 160 * r(12),
    };
  }

  // Simulation, one step per minute, stored the way the monitor stores its records
  const SIZE = (HISTORY_DAYS + 9) * DAY / MIN;
  const rec = {co2: new Uint16Array(SIZE), tmp: new Int16Array(SIZE), hum: new Uint16Array(SIZE), pres: new Uint16Array(SIZE), voc: new Uint16Array(SIZE)};
  const st = {co2: 520, tmp: 20.5, hum: 44, voc: 100, i: 0, learnUntil: S0 + 3 * 3600};
  let planDay = -1, today = null;

  function step() {
    const i = st.i, t = S0 + i * MIN;
    if (i >= SIZE) return false;
    st.i++;
    const d = new Date(t * 1000), h = d.getHours() + d.getMinutes() / 60;
    if (d.getDate() !== planDay) { planDay = d.getDate(); today = plan(d); }
    const p = today;

    const asleep = h < p.wake || h >= p.bed;
    const away = !asleep && h >= p.leave && h < p.back;
    const people = asleep ? p.sleepers : away ? 0 : 2;
    const airing = (h >= p.wake + 0.4 && h < p.wake + 0.4 + 10 / 60) || (p.airEvening > 0 && h >= p.airEvening && h < p.airEvening + p.airMinutes / 60);
    const cooking = p.cook > 0 && h >= p.cook && h < p.cook + 0.7;

    // CO2: 50 m3 room, 0.2 L/min per sleeping and 0.3 per awake person, 0.7 air changes an hour (10 with the window open)
    const outdoor = 420 + 15 * noise(t, 6 * 3600, 1);
    const ach = airing ? 10 : 0.7;
    st.co2 += people * (asleep ? 0.2 : 0.3) * 1000 / 50 - ach / 60 * (st.co2 - outdoor);
    // Temperature: heating 21.3 while someone is up, 19 at night and while away, cold air from the window
    const heat = airing ? 15 : asleep || away ? 19.2 : 21.3 + (cooking ? 0.8 : 0);
    st.tmp += (heat - st.tmp) / (airing ? 20 : 80) +0.004 * Math.max(0, Math.sin(Math.PI * (h - 10) / 8)) * noise(t, DAY, 2);
    // Humidity: people and cooking add, the window takes away
    const wet = airing ? 34 : 40 + 10 * noise(t, 4 * DAY, 3) + people * 1.5 + (cooking ? 8 : 0);
    st.hum += (wet - st.hum) / (airing ? 8 : 45);
    // VOC index: about 100 is the room's usual air; cooking and people raise it, fresh air lowers it
    const vocAim = airing ? 60 : 85 + people * 7 + (cooking ? p.cookVoc : 0) + (noise(t, 5400, 4) > 0.93 ? 140 : 0);
    st.voc += (vocAim - st.voc) / (vocAim > st.voc ? 8 : 25);

    const gap = GAPS.find(g => t >= g.from && t < g.to);
    if (gap) {
      if (gap.learn) st.learnUntil = gap.to + 3 * 3600;
      return true;  // nothing stored, all five stay 0 = missing
    }
    const pres = 975 + 9 * (2 * noise(t, 3.1 * DAY, 5) - 1) + 3 * (2 * noise(t, 0.9 * DAY, 6) - 1) + 0.5 * Math.sin(2 * Math.PI * t / 43200);
    const jitter = (k, a) => (rnd(i, k) - 0.5) * 2 * a;
    rec.co2[i] = Math.max(400, Math.round(st.co2 + jitter(11, 6)));
    rec.tmp[i] = Math.round((st.tmp + jitter(12, 0.03)) * 100);
    rec.hum[i] = Math.round(Math.min(75, Math.max(20, st.hum - 3 * (st.tmp - 21) + jitter(13, 0.3))) * 100);
    rec.pres[i] = Math.round((pres + jitter(14, 0.06)) * 10);
    rec.voc[i] = Math.min(500, Math.max(1, Math.round(st.voc + jitter(15, 3)))) | (t < st.learnUntil ? 0x8000 : 0);
    return true;
  }

  // Simulates up to the newest finished minute
  function catchUp() {
    const end = Math.min(SIZE, (lastMinute() - S0) / MIN + 1);
    while (st.i < end && step()) {}
    return st.i;  // minutes simulated
  }
  const index = t => Math.floor((t - S0) / MIN);

  // ---------- Answers ----------

  function records(res, from, to) {
    const n = catchUp(), out = [];
    if (res === 1) {
      for (let i = Math.max(0, index(Math.max(from, oldest.m1))); i < n && S0 + i * MIN < to; i++) {
        if (S0 + i * MIN >= from && rec.co2[i]) out.push([S0 + i * MIN, rec.co2[i], rec.tmp[i], rec.hum[i], rec.pres[i], rec.voc[i]]);
      }
    } else {
      // 5-minute values: the average of the minutes in each 5 minutes, learning if any minute was
      for (let t = Math.max(from, S0) - Math.max(from, S0) % 300; t < to; t += 300) {
        if (t < from) continue;
        const a = index(t);
        if (a + 5 > n) break;  // the newest 5 minutes aren't finished yet
        let k = 0, c = 0, tm = 0, hu = 0, pr = 0, vo = 0, learn = 0;
        for (let i = a; i < a + 5; i++) {
          if (!rec.co2[i]) continue;
          k++; c += rec.co2[i]; tm += rec.tmp[i]; hu += rec.hum[i]; pr += rec.pres[i]; vo += rec.voc[i] & 0x7fff; learn |= rec.voc[i] & 0x8000;
        }
        if (k) out.push([t, Math.round(c / k), Math.round(tm / k), Math.round(hu / k), Math.round(pr / k), Math.round(vo / k) | learn]);
      }
    }
    const buf = new ArrayBuffer(out.length * 14), v = new DataView(buf);
    out.forEach((r, j) => {
      const o = j * 14;
      v.setUint32(o, r[0], true); v.setUint16(o + 4, r[1], true); v.setInt16(o + 6, r[2], true);
      v.setUint16(o + 8, r[3], true); v.setUint16(o + 10, r[4], true); v.setUint16(o + 12, r[5], true);
    });
    return buf;
  }

  function days(from, to) {
    const n = catchUp(), out = [];
    for (let day = utcDay(from); day < to; day += DAY) {
      let first = 0, last = 0;
      for (let i = Math.max(0, index(day)); i < Math.min(n, index(day + DAY)); i++) {
        if (!rec.co2[i]) continue;
        if (!first) first = S0 + i * MIN;
        last = S0 + i * MIN;
      }
      if (!first) continue;
      const fine = day >= oldest.m1;
      out.push({day, res: fine ? 1 : 5, first: fine ? first : first - first % 300, last: fine ? last : last - last % 300});
    }
    return out;
  }

  function info() {
    const n = catchUp();
    let stored = 0;
    for (let i = 0; i < n; i++) if (rec.co2[i]) stored++;
    const m1 = Math.max(0, stored - (oldest.m1 - S0) / MIN);
    return {on: true, clock: true, usedKB: Math.round((m1 + stored / 5) * 14 / 1024) + 16, totalKB: 10112, unsaved: Math.floor(Date.now() / 60000) % 10,
      oldest: {m1: oldest.m1, m5: oldest.m5}, keepDays: {m1: KEEP.m1, m5: KEEP.m5}};
  }

  const device = {sleep: false, auto: true, level: 50};
  function lightNow() {
    const t = Date.now() / 1000, d = new Date(), h = d.getHours() + d.getMinutes() / 60, p = plan(d);
    const sun = Math.max(0, Math.sin(Math.PI * (h - 7.5) / 10));
    const up = !(h < p.wake || h >= p.bed) && !(h >= p.leave && h < p.back);
    return Math.max(sun * (40 + 360 * noise(t, 1800, 7)), up && h > 16.5 ? 110 + 30 * noise(t, 600, 8) : 0.2);
  }
  const autoLevel = lux => Math.round(Math.min(100, Math.max(1, 5 + 30 * Math.log10(1 + lux))));

  function now() {
    const n = catchUp(), i = n - 1, t = Math.floor(Date.now() / 1000), j = k => (Math.random() - 0.5) * k;
    const lux = lightNow();
    return {time: t, clock: true, uptime: Math.round(t - restartAt), warmup: 0,
      co2: Math.round(rec.co2[i] + j(8)), temperature: +(rec.tmp[i] / 100 + j(0.04)).toFixed(2), humidity: +(rec.hum[i] / 100 + j(0.3)).toFixed(1),
      pressure: +(rec.pres[i] / 10 + j(0.06)).toFixed(2), light: +lux.toFixed(1), voc: Math.max(1, Math.round((rec.voc[i] & 0x7fff) + j(4))),
      vocLearnLeft: Math.max(0, Math.round(st.learnUntil - t)), vocLimits: VOC_LIMITS, sleep: device.sleep,
      brightness: device.auto ? autoLevel(lux) : device.level, autoBrightness: device.auto, limits: LIMITS, rssi: -46 - Math.round(Math.random() * 5)};
  }

  function answer(path, q, method) {
    const json = x => [JSON.stringify(x), 'application/json', 200];
    const bad = text => [text, 'text/plain', 400];
    const span = max => {
      const from = +q.get('from') || 0, to = +q.get('to') || 0;
      return from === 0 || to <= from || to - from > max ? null : [from, to];
    };
    if (path === '/api/now' && method === 'GET') return json(now());
    if (path === '/api/info' && method === 'GET') return json(info());
    if (path === '/api/days' && method === 'GET') {
      const s = span(MAX_SPAN.days);
      return s ? json(days(...s)) : bad('Needs from and to (UTC seconds), at most 45 days apart');
    }
    if (path === '/api/data' && method === 'GET') {
      const res = +q.get('res');
      if (res !== 1 && res !== 5) return bad('res must be 1 or 5');
      const s = span(MAX_SPAN[res]);
      return s ? [records(res, ...s), 'application/octet-stream', 200] : bad('Needs from and to (UTC seconds), at most ' + MAX_SPAN[res] / DAY + ' days apart');
    }
    if (path === '/api/sleep' && method === 'POST') {
      if (q.get('on') !== '0' && q.get('on') !== '1') return bad('on must be 0 or 1');
      device.sleep = q.get('on') === '1';
      return json({sleep: device.sleep});
    }
    if (path === '/api/brightness' && method === 'POST') {
      const level = q.get('level');
      if (q.get('auto') === '1') device.auto = true;
      else if (/^\d{1,3}$/.test(level || '') && +level <= 100) { device.auto = false; device.level = +level; }
      else return bad('Needs auto=1 or level=0..100');
      return json({auto: device.auto, brightness: device.auto ? autoLevel(lightNow()) : device.level});
    }
    return ['Not found', 'text/plain', 404];
  }

  // Requests to /api/... are answered here, after a short delay like the real monitor's
  const realFetch = window.fetch.bind(window);
  window.fetch = (input, options) => {
    const url = new URL(typeof input === 'string' ? input : input.url, location.href), at = url.pathname.indexOf('/api/');
    if (at < 0) return realFetch(input, options);
    const [body, type, status] = answer(url.pathname.slice(at), url.searchParams, ((options && options.method) || 'GET').toUpperCase());
    return new Promise(done => setTimeout(() => done(new Response(body, {status, headers: {'Content-Type': type}})), 60 + Math.random() * 140));
  };

  // A note at the top, so nobody takes the values for real ones
  document.addEventListener('DOMContentLoaded', () => {
    const note = document.createElement('div');
    note.className = 'card';
    note.style.cssText = 'margin-bottom:12px;padding:8px 14px;font-size:13px;color:var(--sub);border-style:dashed';
    note.innerHTML = '<b style="color:var(--ink)">Mockup</b> with made-up values, for trying the page without a monitor. Nothing is sent anywhere: the sleep switch and the brightness only change this copy.';
    document.querySelector('main').prepend(note);
  });
})();
