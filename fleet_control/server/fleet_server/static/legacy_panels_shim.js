'use strict';
// G2 2026-11-23: globals expected by the origin/main panels (operations_panel, gamepad_panel, camera_panel).
// The console app.js keeps its own scope; this shim only reads the same session token and the same API.
const API = '/api/v1';
let token = sessionStorage.getItem('fleet_token') || '', principalName = '', data = null;
const $ = id => document.getElementById(id);
async function api(path, opt = {}) {
  token = sessionStorage.getItem('fleet_token') || '';
  return fetch(API + path, {...opt, headers: {'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json', ...(opt.headers || {})}});
}
(async function refresh() {
  try {
    token = sessionStorage.getItem('fleet_token') || '';
    if (token) {
      if (!principalName) { const r = await api('/me'); if (r.ok) principalName = (await r.json()).name || ''; }
      const r = await api('/dashboard'); if (r.ok) data = await r.json();
    }
  } catch (e) { /* the console shows connection errors */ }
  setTimeout(refresh, 1000);
})();
const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c]));
function fillSelect(sel, ids) {
  if (!sel) return;
  const cur = sel.value;
  if (sel.options.length !== ids.length || [...sel.options].some((o, i) => o.value !== ids[i])) {
    sel.innerHTML = ids.map(i => `<option>${esc(i)}</option>`).join('');
    if (ids.includes(cur)) sel.value = cur;
  }
}
const uuid = () => ([1e7] + -1e3 + -4e3 + -8e3 + -1e11).replace(/[018]/g, c => (c ^ (Math.random() * 16) >> c / 4).toString(16));
// origin/main redraws its own map canvas after the robot selection changes. The console owns its map view and re-reads the
// #mapRobot select on every refresh tick (app.js), so nothing has to be drawn here. (Dispatching a change event instead would
// call selectOperationRobot again through its onchange handler and loop.)
function drawMap() {}
