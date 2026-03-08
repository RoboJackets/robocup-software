import * as t from "ws";
import { AbstractTransport as s } from "./RosLib.js";
class i extends s {
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
    return this.socket.readyState === t.WebSocket.CONNECTING;
  }
  isOpen() {
    return this.socket.readyState === t.WebSocket.OPEN;
  }
  isClosing() {
    return this.socket.readyState === t.WebSocket.CLOSING;
  }
  isClosed() {
    return this.socket.readyState === t.WebSocket.CLOSED;
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
  i as WsWebSocketTransport
};
