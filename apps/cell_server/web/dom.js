// The two helpers every view needs. Four copies of an HTML escape is four
// chances for one of them to be the wrong one.
export const el = id => document.getElementById(id);

export const esc = s =>
  String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
