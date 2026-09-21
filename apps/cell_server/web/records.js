import { esc } from './dom.js';

// A list of records the user edits in place. Both editors have this shape — an
// application's steps, a scene's objects — and only the fields differ, so the
// row scaffolding (number, move, remove) and the rebinding after every render
// are written once here.
//
// Typing must not re-render: an input edits the model in place, because
// rebuilding the DOM under a cursor moves it.

const cell = (field, value, index) => {
  const id = `${field.key}`;
  if (field.type === 'select') {
    const options = field.options ?? [];
    // An unset argument must stay unset: without a blank option, opening the
    // editor silently writes the first station into every step that had none.
    const blank = options.includes(value) ? '' : `<option value=""${value ? '' : ' selected'}>—</option>`;
    return `<label class="argf">${esc(field.label ?? id)}
      <select data-i="${index}" data-field="${esc(id)}" aria-label="${esc(field.label ?? id)} ${index + 1}">
        ${blank}${options.map(o =>
          `<option value="${esc(o)}"${o === value ? ' selected' : ''}>${esc(o)}</option>`).join('')}
      </select></label>`;
  }
  if (field.type === 'bool') {
    return `<label class="argf">${esc(field.label ?? id)}
      <input type="checkbox" data-i="${index}" data-field="${esc(id)}"
        ${value ? 'checked' : ''} aria-label="${esc(field.label ?? id)} ${index + 1}" /></label>`;
  }
  if (field.type === 'vec3') {
    return (value ?? [0, 0, 0]).map((v, axis) =>
      `<label class="argf">${esc(field.label ?? id)}${'xyz'[axis]}
        <input type="number" step="0.01" data-i="${index}" data-field="${esc(id)}" data-axis="${axis}"
          value="${esc(v)}" aria-label="${esc(field.label ?? id)} ${'xyz'[axis]} ${index + 1}" /></label>`).join('');
  }
  return `<label class="argf">${esc(field.label ?? id)}
    <input ${field.type === 'number' ? 'type="number" step="0.01"' : ''} data-i="${index}"
      data-field="${esc(id)}" value="${esc(value ?? '')}"
      aria-label="${esc(field.label ?? id)} ${index + 1}" /></label>`;
};

// `fields` may be a list, or a function of the record when the row's fields
// depend on what it is (an application step's arguments depend on its verb).
// The row is a four-column grid, so it always emits four cells: a record type
// with no lead control (a scene object) still renders an empty one. Skipping it
// shifted the fields into the fixed column and the controls into the flexible
// one, and the row grew past the panel — visible only as three buttons where
// there should be four.
export function renderRecords(host, records, fields, hooks = {}) {
  const { onChange, onStructure, empty = '', lead } = hooks;
  host.innerHTML = records.map((record, i) => {
    const spec = typeof fields === 'function' ? fields(record) : fields;
    const body = spec.map(f => cell(f, record[f.key], i)).join('');
    return `<div class="step" data-i="${i}">
      <span class="stepno">${i + 1}</span>
      <span class="steplead">${lead ? lead(record, i) : ''}</span>
      <div class="args">${body || `<span class="drv">${esc(empty)}</span>`}</div>
      <div class="stepops">
        <button class="stepup" data-i="${i}" aria-label="move ${i + 1} up">↑</button>
        <button class="stepdown" data-i="${i}" aria-label="move ${i + 1} down">↓</button>
        <button class="stepdel" data-i="${i}" aria-label="remove ${i + 1}">✕</button>
      </div>
    </div>`;
  }).join('') || `<span class="drv">${esc(empty)}</span>`;

  const at = e => Number(e.currentTarget.dataset.i);
  const read = input =>
    input.type === 'checkbox' ? input.checked
      : input.type === 'number' ? Number(input.value)
      : input.value;

  host.querySelectorAll('.args input, .args select').forEach(input => {
    const handler = e => {
      const target = e.currentTarget;
      const { field, axis } = target.dataset;
      const record = records[at(e)];
      if (axis === undefined) record[field] = read(target);
      else (record[field] ??= [0, 0, 0])[Number(axis)] = read(target);
      onChange?.(records);
    };
    if (input.tagName === 'SELECT' || input.type === 'checkbox') input.onchange = handler;
    else input.oninput = handler;
  });

  const restructure = fn => e => {
    fn(at(e));
    onStructure?.(records);
  };
  host.querySelectorAll('.stepdel').forEach(b => b.onclick = restructure(i => records.splice(i, 1)));
  host.querySelectorAll('.stepup').forEach(b => b.onclick = restructure(i => move(records, i, -1)));
  host.querySelectorAll('.stepdown').forEach(b => b.onclick = restructure(i => move(records, i, 1)));
}

function move(records, i, by) {
  const j = i + by;
  if (j < 0 || j >= records.length) return;
  [records[i], records[j]] = [records[j], records[i]];
}

// A list of SAVED documents — an algorithm you wrote, a comparison you kept.
// Same three parts every time: what it is called, a way to reopen it, a way to
// throw it away. It was written twice, differing only in which collection it
// read and what "load" meant; the third would have been the one that forgot the
// delete button's label.
export function renderLibrary(host, items, { name, load, onLoad, onDelete, empty }) {
  if (!host) return;
  host.innerHTML = items.map(item => {
    const label = name(item);
    return `<div class="libitem"><span class="libname">${esc(label)}</span>
      <button class="libload" data-file="${esc(item.file)}">${esc(load)}</button>
      <button class="libdel" data-file="${esc(item.file)}"
        aria-label="delete ${esc(label)}">✕</button></div>`;
  }).join('') || `<span class="drv">${esc(empty)}</span>`;

  host.querySelectorAll('.libload').forEach(b => { b.onclick = () => onLoad(b.dataset.file); });
  host.querySelectorAll('.libdel').forEach(b => { b.onclick = () => onDelete(b.dataset.file); });
}
