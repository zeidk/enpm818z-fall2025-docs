// _static/no_auto_theme.js
(function () {
  const KEY = "theme";
  const html = document.documentElement;

  // 1) Normalize any previously stored "auto" to a concrete theme
  try {
    const saved = localStorage.getItem(KEY);
    if (saved === "auto") {
      // pick your default here: "light" or "dark"
      localStorage.setItem(KEY, "light");
      html.setAttribute("data-theme", "light");
    } else if (saved === "light" || saved === "dark") {
      html.setAttribute("data-theme", saved);
    } else {
      // no saved theme: choose a default (no auto)
      const DEFAULT = "light"; // change to "dark" if you prefer
      localStorage.setItem(KEY, DEFAULT);
      html.setAttribute("data-theme", DEFAULT);
    }
  } catch (_) {}

  // 2) Replace Furo’s 3-state cycle with a 2-state toggle (light <-> dark)
  function installTwoStateToggle() {
    const btn = document.querySelector("button.theme-toggle");
    if (!btn) return;

    // Remove Furo’s default behavior by stopping the original handler
    btn.addEventListener(
      "click",
      function (ev) {
        ev.stopImmediatePropagation();
        ev.preventDefault();

        const cur = html.getAttribute("data-theme");
        const next = cur === "dark" ? "light" : "dark";
        html.setAttribute("data-theme", next);
        try {
          localStorage.setItem(KEY, next);
        } catch (_) {}

        // Optional: update aria-label so tooltip doesn’t mention Auto
        btn.setAttribute(
          "aria-label",
          next === "dark" ? "Switch to light theme" : "Switch to dark theme"
        );
        btn.title = btn.getAttribute("aria-label");
      },
      true // capture to beat Furo's listener
    );

    // Optional: set initial accessible label
    const cur = html.getAttribute("data-theme");
    btn.setAttribute(
      "aria-label",
      cur === "dark" ? "Switch to light theme" : "Switch to dark theme"
    );
    btn.title = btn.getAttribute("aria-label");
  }

  // Run after DOM is ready
  if (document.readyState !== "loading") installTwoStateToggle();
  else document.addEventListener("DOMContentLoaded", installTwoStateToggle);
})();
