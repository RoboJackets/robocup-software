import { AbstractTransport as t } from "./RosLib.js";
class r extends t {
  constructor(e) {
    super(), this.socket = e, this.registerEventListeners();
  }
  send(e) {
    this.socket.send(JSON.stringify(e));
  }
  close() {
    this.socket.close();
  }
  isConnecting() {
    return this.socket.readyState === WebSocket.CONNECTING;
  }
  isOpen() {
    return this.socket.readyState === WebSocket.OPEN;
  }
  isClosing() {
    return this.socket.readyState === WebSocket.CLOSING;
  }
  isClosed() {
    return this.socket.readyState === WebSocket.CLOSED;
  }
  registerEventListeners() {
    this.socket.onopen = (e) => {
      this.emit("open", e);
    }, this.socket.onclose = (e) => {
      this.emit("close", e);
    }, this.socket.onerror = (e) => {
      this.emit("error", e);
    }, this.socket.onmessage = (e) => {
      this.handleRawMessage(e.data);
    };
  }
}
export {
  r as NativeWebSocketTransport
};
