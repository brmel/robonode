// WAI-ARIA tabs: roles, aria-selected, roving tabindex, arrow-key navigation.
export function initDock() {
  const tabs = document.getElementById('docktabs');
  if (!tabs) return { show: () => {} };
  const buttons = [...tabs.querySelectorAll('[role="tab"]')];
  const panels = [...document.querySelectorAll('#dock [role="tabpanel"]')];

  const show = name => {
    buttons.forEach(b => {
      const on = b.dataset.tab === name;
      b.classList.toggle('on', on);
      b.setAttribute('aria-selected', String(on));
      b.tabIndex = on ? 0 : -1;
    });
    panels.forEach(p => {
      const on = p.dataset.panel === name;
      p.classList.toggle('on', on);
      p.hidden = !on;
    });
  };

  buttons.forEach((b, i) => {
    b.onclick = () => show(b.dataset.tab);
    b.onkeydown = e => {
      const step = e.key === 'ArrowRight' ? 1 : e.key === 'ArrowLeft' ? -1 : 0;
      if (!step) return;
      e.preventDefault();
      const next = buttons[(i + step + buttons.length) % buttons.length];
      show(next.dataset.tab);
      next.focus();
    };
  });
  show(buttons[0]?.dataset.tab);
  return { show };
}
