(() => {
  const status = document.createElement("div");
  status.setAttribute("role", "status");
  status.setAttribute("aria-live", "polite");
  Object.assign(status.style, {
    background: "#ffffff",
    border: "1px solid #b7c0c7",
    bottom: "8px",
    color: "#172026",
    display: "none",
    font: "12px -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif",
    left: "8px",
    lineHeight: "1.35",
    maxWidth: "min(720px, calc(100vw - 16px))",
    overflowWrap: "anywhere",
    padding: "6px 8px",
    position: "fixed",
    zIndex: "9999",
  });

  document.body.append(status);
  show("Loading presentation...");

  function show(message) {
    status.textContent = message;
    status.style.display = "block";
  }

  function hide() {
    status.style.display = "none";
  }

  function formatAssetUrl(assetUrl) {
    try {
      const url = new URL(assetUrl, window.location.href);
      const currentPathParts = window.location.pathname.split("/");
      currentPathParts.pop();
      const currentDirectory = `${currentPathParts.join("/")}/`;
      const relativePath = url.pathname.startsWith(currentDirectory)
        ? url.pathname.slice(currentDirectory.length)
        : url.pathname;

      return decodeURI(`${relativePath}${url.search}${url.hash}`);
    } catch {
      return assetUrl;
    }
  }

  function getMediaUrl(element) {
    return element.currentSrc || element.src || element.getAttribute("src");
  }

  function isMediaLoaded(element) {
    if (element instanceof HTMLImageElement) {
      return element.complete;
    }

    if (element instanceof HTMLVideoElement) {
      return element.readyState >= HTMLMediaElement.HAVE_METADATA;
    }

    return true;
  }

  window.brickSimLoadingStatus = {
    hide,
    show,
    trackMedia(root) {
      const pendingMedia = new Map();

      function showNextPending() {
        const nextUrl = pendingMedia.values().next().value;

        if (nextUrl) {
          show(`Loading ${formatAssetUrl(nextUrl)}`);
        } else {
          hide();
        }
      }

      for (const element of root.querySelectorAll("img, video")) {
        const assetUrl = getMediaUrl(element);

        if (!assetUrl || isMediaLoaded(element)) {
          continue;
        }

        const finish = () => {
          pendingMedia.delete(element);
          showNextPending();
        };

        pendingMedia.set(element, assetUrl);
        element.addEventListener("load", finish, { once: true });
        element.addEventListener("loadedmetadata", finish, { once: true });
        element.addEventListener("error", finish, { once: true });
      }

      showNextPending();
    },
  };
})();
