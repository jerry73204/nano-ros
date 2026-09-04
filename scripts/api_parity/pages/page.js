const DATA = window.__PARITY__;
const STATES = [
  ["same","same","●"],["reshaped","re-shaped","●"],["renamed","renamed","●"],
  ["remapped","re-mapped","◆"],["rejected","rejected","✕"],
  ["missing","not implemented","○"],["ours","ours only","✥"]
];
const GLYPH = Object.fromEntries(STATES.map(s => [s[0], s[2]]));
const LABEL = Object.fromEntries(STATES.map(s => [s[0], s[1]]));
// RFC-0089 dispositions. A SECOND axis, not an eighth state: the state says
// what we did, the disposition says what a porting user gets.
const DISP = {
  "adopt": "adopt", "adopt-bounded": "adopt \u00b7 bounded",
  "refuse-loud": "refuses loudly", "absent": "absent"
};
const on = new Set(STATES.map(s => s[0]));
let query = "";
let alias = false;   // render our side under ROS 2's spelling

const esc = s => String(s == null ? "" : s)
  .replace(/&/g,"&amp;").replace(/</g,"&lt;").replace(/>/g,"&gt;");
// ledger prose is authored with `backticks`; escape first, then mark up
const prose = s => esc(s).replace(/`([^`]+)`/g, (_, c) => "<code>" + c + "</code>");

/* Format A, the RMW reference's shape: return type on its own line, then the
   name, then ONE ARGUMENT PER LINE. Collapsed to a single line at zero or one
   argument, where the vertical form buys nothing. The two signature cells stay
   line-for-line comparable, which is why the status chip heads the reason
   column instead of sitting in the nano-ros cell. */
function cellHTML(cell, renamed, ours) {
  if (!cell) return '<div class="nosig">— none</div>';
  // `a`/`ar`/`ap` are the same signature spelled the way ROS 2 spells it —
  // a VIEW for comparing shape, never a claim about what the code is called.
  const useAlias = ours && alias && cell.a;
  const nm = '<span class="fn' + (renamed && !useAlias ? " ren" : "") +
             (useAlias ? " aliased" : "") + '">' +
             esc(useAlias ? cell.a : cell.n) + "</span>";
  if (!cell.p) {
    const k = cell.k ? '<span class="kind">' + esc(cell.k) + "</span> " : "";
    return "<pre>" + k + nm + "</pre>";
  }
  const rawRet = useAlias && cell.ar ? cell.ar : cell.r;
  const params = useAlias && cell.ap ? cell.ap : cell.p;
  const ret = rawRet ? '<span class="ret">' + esc(rawRet) + "</span>\n" : "";
  const arg = ([txt, flag]) =>
    '<span class="ty' + (flag ? " " + flag : "") + '">' + esc(txt) + "</span>";
  let body;
  if (!params.length) body = nm + '<span class="pu">(</span><span class="ty">void</span><span class="pu">)</span>';
  else if (params.length === 1) body = nm + '<span class="pu">(</span>' + arg(params[0]) + '<span class="pu">)</span>';
  else body = nm + '<span class="pu">(</span>\n' +
       params.map(p => "  " + arg(p)).join('<span class="pu">,</span>\n') +
       '\n<span class="pu">)</span>';
  const more = cell.x ? '\n<span class="pu">/* +' + cell.x +
    " overload" + (cell.x === 1 ? "" : "s") + " */</span>" : "";
  return "<pre>" + ret + body + more + "</pre>";
}

function searchText(r) {
  if (r._s === undefined) {
    const bits = [r.k, r.w, r.d || "", r.su === "ported" ? "compat shim rclcpp_compat" : ""];
    [r.T, r.O].forEach(c => {
      if (!c) return;
      bits.push(c.n, c.r || "");
      (c.p || []).forEach(p => bits.push(p[0]));
    });
    (r.p || []).forEach(x => bits.push(x));
    r._s = bits.join(" ").toLowerCase();
  }
  return r._s;
}

function match(r) {
  if (!on.has(r.s)) return false;
  return !query || searchText(r).includes(query.toLowerCase());
}

function rowHTML(r) {
  let why = "";
  why += '<div class="st s-' + r.s + '">' + GLYPH[r.s] + " " + LABEL[r.s] + "</div>";
  // issue 1020: our side of this row exists only inside `rclcpp_compat.hpp`, or
  // the native headers answer it differently. Either way the reader is told,
  // rather than reading one merged number for two questions.
  if (r.su === "ported" || r.nb) {
    let bits = [];
    if (r.su === "ported") bits.push("via <code>rclcpp_compat.hpp</code>");
    if (r.nb) bits.push("native API: <b>" + esc(r.nb) + "</b>");
    why += '<div class="surf">' + bits.join(" \u00b7 ") + "</div>";
  }
  if (r.d) {
    why += '<div class="disp d-' + esc(r.d) + '">' +
      esc(DISP[r.d] || r.d) + "</div>";
  }
  if (r.p && r.p.length) {
    why += '<div class="answers">' +
      r.p.map(x => '<div class="ans"><code>' + esc(x) + "</code></div>").join("") + "</div>";
  }
  if (r.w) why += '<div class="wtext">' + prose(r.w) + "</div>";
  return "<tr" + (r.i ? ' class="inh"' : "") + ">" +
    '<td class="c" data-lbl="ROS 2">' + cellHTML(r.T, false, false) + "</td>" +
    '<td class="c" data-lbl="nano-ros">' + cellHTML(r.O, !!r.ren, true) + "</td>" +
    '<td class="why">' + why + "</td></tr>";
}

function groupBlock(gname, rows) {
  const states = [...new Set(rows.map(r => r.s))];
  const uniform = states.length === 1;
  const withWhy = rows.filter(r => r.w && !r.i);
  let h = '<details class="grp"' + (rows.length <= 6 ? " open" : "") + ">";
  h += "<summary>";
  h += '<span class="gname">' + esc(gname) + "</span>";
  h += '<span class="gmeta">' + rows.length + (rows.length === 1 ? " item" : " items") + "</span>";
  h += '<span class="gchips">' + STATES.filter(s => states.includes(s[0])).map(s =>
        '<span class="tag ' + s[0] + '">' + s[2] + " " + s[1] +
        (uniform ? "" : " " + rows.filter(r => r.s === s[0]).length) + "</span>").join("") +
       "</span></summary>";
  // a whole group that shares one verdict states its reason ONCE, rather than
  // repeating it down every member row
  if (uniform && withWhy.length) {
    h += '<p class="uniform"><em>All ' + rows.length + " " + LABEL[states[0]] + ".</em> " +
         prose(withWhy[0].w) + "</p>";
  }
  h += '<div class="tablewrap"><table><colgroup><col class="k1"><col class="k2"><col class="k3"></colgroup>' +
       "<thead><tr><th>ROS 2</th><th>nano-ros</th><th>verdict &amp; reason</th></tr></thead><tbody>" +
       rows.map(rowHTML).join("") + "</tbody></table></div>";
  return h + "</details>";
}

function render() {
  const rows = DATA.rows.filter(match);
  document.getElementById("shown").textContent = rows.length;
  const host = document.getElementById("body");
  if (!rows.length) { host.innerHTML = '<p class="empty-note">No items match this filter.</p>'; return; }
  let h = "";
  const push = (map, key, r) => { if (!map.has(key)) map.set(key, []); map.get(key).push(r); };
  if (DATA.layout === "flat") {
    const by = new Map();
    rows.forEach(r => push(by, r.g, r));
    [...by.keys()].sort().forEach(g => { h += groupBlock(g, by.get(g)); });
  } else {
    const secs = new Map();
    rows.forEach(r => {
      if (!secs.has(r.sec)) secs.set(r.sec, new Map());
      push(secs.get(r.sec), r.g, r);
    });
    const total = m => [...m.values()].reduce((n, v) => n + v.length, 0);
    [...secs.keys()].sort((a, b) => total(secs.get(b)) - total(secs.get(a))).forEach(sec => {
      const m = secs.get(sec);
      h += '<section class="aud"><h2>' + esc(sec) +
           '<span class="c">' + total(m) + " items · " + m.size + " groups</span></h2>";
      [...m.keys()].sort().forEach(g => { h += groupBlock(g, m.get(g)); });
      h += "</section>";
    });
  }
  host.innerHTML = h;
}

function buildChips() {
  const host = document.getElementById("chips");
  host.innerHTML = STATES.map(([k, l, g]) =>
    '<button class="chip ' + k + '" data-k="' + k + '" data-on="1" type="button" ' +
    'aria-pressed="true"><span class="g">' + g + '</span>' + l +
    ' <span class="n">' + (DATA.counts[k] || 0) + "</span></button>").join("");
  host.addEventListener("click", e => {
    const b = e.target.closest(".chip"); if (!b) return;
    const k = b.dataset.k;
    if (on.has(k) && on.size === STATES.length) { on.clear(); on.add(k); }
    else if (on.has(k)) { on.delete(k); if (!on.size) STATES.forEach(s => on.add(s[0])); }
    else on.add(k);
    host.querySelectorAll(".chip").forEach(c => {
      const v = on.has(c.dataset.k) ? "1" : "0";
      c.dataset.on = v; c.setAttribute("aria-pressed", v === "1" ? "true" : "false");
    });
    render();
  });
}

buildChips();
document.getElementById("q").addEventListener("input", e => { query = e.target.value; render(); });
const aliasBtn = document.getElementById("alias");
aliasBtn.addEventListener("click", () => {
  alias = !alias;
  aliasBtn.dataset.on = alias ? "1" : "0";
  aliasBtn.setAttribute("aria-pressed", alias ? "true" : "false");
  render();
});
document.getElementById("expand").addEventListener("click", () => {
  const allOpen = !document.querySelector(".grp:not([open])");
  document.querySelectorAll(".grp").forEach(d => { d.open = !allOpen; });
});
render();
