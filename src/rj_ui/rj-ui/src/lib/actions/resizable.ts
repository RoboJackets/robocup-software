// lib/actions/resizable.ts
export function resizable(
    node: HTMLElement,
    params: { minWidth: number, maxWidth: number, initialWidth: number} = { minWidth:  200, maxWidth: 800, initialWidth: 280 }
) {
  let width = params.initialWidth;
  node.style.width = `${width}px`;

  const handle = document.createElement('div');
  // Styling the invisible "grab" area
  handle.style.cssText = `
    width: 6px;
    height: 100%;
    cursor: col-resize;
    position: absolute;
    right: -3px;
    top: 0;
    z-index: 50;
  `;
  node.appendChild(handle);

  function onMouseDown(e: MouseEvent) {
    e.preventDefault();
    window.addEventListener('mousemove', onMouseMove);
    window.addEventListener('mouseup', onMouseUp);
    document.body.style.cursor = 'col-resize';
  }

  function onMouseMove(e: MouseEvent) {
    const newWidth = e.clientX;
    if (newWidth >= params.minWidth && newWidth <= params.maxWidth) {
      width = newWidth;
      node.style.width = `${width}px`;
    }
  }

  function onMouseUp() {
    window.removeEventListener('mousemove', onMouseMove);
    window.removeEventListener('mouseup', onMouseUp);
    document.body.style.cursor = 'default';
  }

  handle.addEventListener('mousedown', onMouseDown);

  return {
    destroy() {
      handle.removeEventListener('mousedown', onMouseDown);
    }
  };
}