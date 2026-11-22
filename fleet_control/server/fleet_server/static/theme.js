'use strict';
(() => {
  const key = 'limo_fleet_theme_v1';
  const options = ['system', 'light', 'dark'];
  const media = window.matchMedia('(prefers-color-scheme: dark)');
  let choice = 'system';
  try { const stored = localStorage.getItem(key); if (options.includes(stored)) choice = stored; } catch (_) {}
  function apply() {
    document.documentElement.dataset.theme = choice === 'system' ? (media.matches ? 'dark' : 'light') : choice;
    document.documentElement.dataset.themeChoice = choice;
    for (const id of ['themeSelect', 'themeSettings']) { const el = document.getElementById(id); if (el) el.value = choice; }
    window.dispatchEvent(new CustomEvent('fleet-theme-change'));
  }
  for (const id of ['themeSelect', 'themeSettings']) {
    document.getElementById(id).addEventListener('change', e => {
      if (!options.includes(e.target.value)) return;
      choice = e.target.value;
      try { localStorage.setItem(key, choice); } catch (_) {}
      apply();
    });
  }
  media.addEventListener('change', () => { if (choice === 'system') apply(); });
  apply();
})();
