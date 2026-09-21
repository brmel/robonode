// A ~40-line observable store: one state object, selector subscriptions.
// Views re-render only when the slice they asked for actually changes, which
// is what stops a 50 Hz telemetry stream from rebuilding the whole DOM.
export function createStore(initial = {}) {
  let state = initial;
  const subs = new Set();

  const get = () => state;

  const patch = partial => {
    const next = { ...state, ...partial };
    const changed = Object.keys(partial).some(k => !Object.is(state[k], partial[k]));
    if (!changed) return;
    state = next;
    subs.forEach(s => s.run(state));
  };

  const subscribe = (selector, fn) => {
    const sub = {
      last: selector(state),
      run(s) {
        const value = selector(s);
        if (same(value, sub.last)) return;
        sub.last = value;
        fn(value, s);
      },
    };
    subs.add(sub);
    fn(sub.last, state);
    return () => subs.delete(sub);
  };

  return { get, patch, subscribe };
}

// Shallow equality is enough: telemetry slices are numbers, strings, or arrays
// of numbers rebuilt each frame.
function same(a, b) {
  if (Object.is(a, b)) return true;
  if (Array.isArray(a) && Array.isArray(b)) {
    return a.length === b.length && a.every((v, i) => Object.is(v, b[i]));
  }
  return false;
}
