// Mockup of the monitor (docs section 8.7): answers the page's /api/... requests in the
// browser with made-up values, so the page can be seen without a monitor.
// make_web_page.ps1 puts it into docs/web_page_mockup.html, in front of the page's own script.
(function () {
  'use strict';
  const DAY = 86400, START = Math.floor(Date.now() / 1000 / DAY) * DAY - 6 * DAY;  // 7 days of values
  const nowS = () => Math.floor(Date.now() / 1000);
  const wiggle = t => { const x = Math.sin(t * 12.9898) * 43758.5453; return x - Math.floor(x) - 0.5; };
  const daily = (h, peak) => Math.cos(2 * Math.PI * (h - peak) / 24);  // 1 at the peak hour, -1 twelve hours later

  // Made-up values at a time: a daily rhythm plus a little noise
  function values(t) {
    const d = new Date(t * 1000), h = d.getHours() + d.getMinutes() / 60;
    return {
      co2: 800 + 350 * daily(h, 3) + 60 * Math.sin(t / 2000) + 10 * wiggle(t),            // high at night
      tmp: 21 + 0.8 * daily(h, 16) + 0.05 * wiggle(t + 1),
      hum: 47 + 4 * daily(h, 4) + 0.3 * wiggle(t + 2),
      pres: 975 + 3 * Math.sin(t / 30000) + 0.05 * wiggle(t + 3),
      voc: 100 + 150 * Math.exp(-Math.pow((h - 19) / 0.6, 2)) + 15 * Math.sin(t / 3000) + 3 * wiggle(t + 4),  // cooking at 19:00
    };
  }

  // Stored values as the monitor sends them: 14 bytes per minute or 5 minutes
  function records(res, from, to) {
    const step = res * 60, list = [];
    for (let t = Math.ceil(Math.max(from, START) / step) * step; t < to && t + step <= nowS(); t += step) list.push(t);
    const buf = new ArrayBuffer(list.length * 14), v = new DataView(buf);
    list.forEach((t, i) => {
      const x = values(t + step / 2), o = i * 14;
      v.setUint32(o, t, true);
      v.setUint16(o + 4, Math.round(x.co2), true);
      v.setInt16(o + 6, Math.round(x.tmp * 100), true);
      v.setUint16(o + 8, Math.round(x.hum * 100), true);
      v.setUint16(o + 10, Math.round(x.pres * 10), true);
      v.setUint16(o + 12, Math.round(x.voc), true);
    });
    return buf;
  }

  const device = {sleep: false, auto: true, level: 50};
  function answer(path, q) {
    const t = nowS(), last = t - t % 60 - 60;
    if (path === '/api/now') {
      const x = values(t), j = k => (Math.random() - 0.5) * k;
      return {time: t, clock: true, uptime: t - START, warmup: 0, co2: Math.round(x.co2 + j(10)), temperature: +(x.tmp + j(0.05)).toFixed(2),
        humidity: +(x.hum + j(0.4)).toFixed(1), pressure: +(x.pres + j(0.1)).toFixed(2), light: +(120 + j(10)).toFixed(1), voc: Math.round(x.voc + j(4)),
        vocLearnLeft: 0, vocLimits: [150, 250, 400], sleep: device.sleep, brightness: device.auto ? 40 : device.level,
        autoBrightness: device.auto, limits: [600, 1000, 1500], rssi: -48};
    }
    if (path === '/api/info') {
      return {on: true, clock: true, usedKB: 140, totalKB: 10112, unsaved: 3, oldest: {m1: START, m5: START}, keepDays: {m1: 14, m5: 1096}};
    }
    if (path === '/api/days') {
      const out = [];
      for (let day = START; day <= last; day += DAY) {
        if (day + DAY > +q.get('from') && day < +q.get('to')) out.push({day, res: 1, first: day, last: Math.min(day + DAY - 60, last)});
      }
      return out;
    }
    if (path === '/api/data') return records(+q.get('res'), +q.get('from'), +q.get('to'));
    if (path === '/api/sleep') return {sleep: device.sleep = q.get('on') === '1'};
    if (path === '/api/brightness') {
      device.auto = q.get('auto') === '1';
      if (!device.auto) device.level = +q.get('level');
      return {auto: device.auto, brightness: device.auto ? 40 : device.level};
    }
    return null;
  }

  // Requests to /api/... are answered here instead of by a monitor
  const realFetch = window.fetch.bind(window);
  window.fetch = (input, options) => {
    const url = new URL(typeof input === 'string' ? input : input.url, location.href), at = url.pathname.indexOf('/api/');
    if (at < 0) return realFetch(input, options);
    const a = answer(url.pathname.slice(at), url.searchParams);
    const body = a === null ? 'Not found' : a instanceof ArrayBuffer ? a : JSON.stringify(a);
    return Promise.resolve(new Response(body, {status: a === null ? 404 : 200}));
  };

  // A note at the top, so nobody takes the values for real ones
  document.addEventListener('DOMContentLoaded', () => {
    const note = document.createElement('div');
    note.className = 'card';
    note.style.cssText = 'margin-bottom:12px;padding:8px 14px;font-size:13px;color:var(--sub);border-style:dashed';
    note.innerHTML = '<b style="color:var(--ink)">Mockup</b> with made-up values: this is how the monitor\'s page looks. Nothing is sent anywhere.';
    document.querySelector('main').prepend(note);
  });
})();
