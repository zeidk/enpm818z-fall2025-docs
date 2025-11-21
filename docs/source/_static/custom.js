document.addEventListener("DOMContentLoaded", () => {
  // Select all <img> elements inside figure or image directives
  document.querySelectorAll("img").forEach(img => {
    // Create button
    const button = document.createElement("button");
    button.className = "copy-image-btn";
    button.innerText = "📋 Copy";

    // Add button before the image
    img.insertAdjacentElement("beforebegin", button);

    // Copy handler
    button.addEventListener("click", async () => {
      try {
        const response = await fetch(img.src);
        const blob = await response.blob();
        await navigator.clipboard.write([
          new ClipboardItem({ [blob.type]: blob })
        ]);
        button.innerText = "✅ Copied!";
        setTimeout(() => (button.innerText = "📋 Copy"), 2000);
      } catch (err) {
        console.error("Copy failed:", err);
        button.innerText = "❌ Error";
        setTimeout(() => (button.innerText = "📋 Copy"), 2000);
      }
    });
  });
});
