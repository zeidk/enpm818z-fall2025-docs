/*!
 * Minimal offline pseudocode.js loader
 * https://github.com/codemirror/pseudocode.js (MIT)
 * Adapted for local Sphinx builds
 */
(function () {
  function ready(fn) {
    if (document.readyState !== "loading") fn();
    else document.addEventListener("DOMContentLoaded", fn);
  }
  ready(function () {
    if (window.renderPseudocode) {
      const blocks = document.querySelectorAll("pre.pseudocode");
      for (const block of blocks) {
        try {
          const code = block.textContent.trim();
          block.outerHTML = window.renderPseudocode(code, { indentSize: "1.5em" });
        } catch (err) {
          console.error("Pseudocode render failed:", err);
        }
      }
    }
  });
})();
