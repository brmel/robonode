// The bridge between CSS tokens and everything that is not CSS (the 3D scene).
// One palette, read at runtime, so a theme switch repaints the viewport too.
const KEY = 'robonode-theme';
const listeners = new Set();

const root = document.documentElement;

export const token = name => getComputedStyle(root).getPropertyValue(name).trim();

// A token as a THREE-friendly integer, e.g. color('--scene-bg') -> 0x0e1116.
export const color = name => {
  const raw = token(name);
  return raw.startsWith('#') ? parseInt(raw.slice(1), 16) : 0x000000;
};

// index.html stamps data-theme before paint, so this is always a real answer.
export const current = () => root.getAttribute('data-theme') ?? 'dark';

export function apply(name) {
  root.setAttribute('data-theme', name);
  localStorage.setItem(KEY, name);
  listeners.forEach(fn => fn(name));
}

export const toggle = () => apply(current() === 'dark' ? 'light' : 'dark');
export const onChange = fn => { listeners.add(fn); return () => listeners.delete(fn); };

// The pre-paint script already stamped the attribute; nothing to restore.
export const restore = () => current();
