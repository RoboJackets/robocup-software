import { MAX_HIGH_SURROGATE, MIN_HIGH_SURROGATE } from './const.js';
import { Wtf8Encoder } from './encode.js';
function isHighSurrogate(n) {
    return (MIN_HIGH_SURROGATE <= n) && (n <= MAX_HIGH_SURROGATE);
}
export class Wtf8EncoderStream {
    #pendingHighSurrogate = null;
    #handle;
    #transform;
    constructor() {
        this.#handle = new Wtf8Encoder();
        this.#transform = new TransformStream({
            transform: (chunk, controller) => {
                chunk = String(chunk);
                if (this.#pendingHighSurrogate != null) {
                    chunk = this.#pendingHighSurrogate + chunk;
                    this.#pendingHighSurrogate = null;
                }
                if (isHighSurrogate(chunk.charCodeAt(chunk.length - 1))) {
                    this.#pendingHighSurrogate = chunk[chunk.length - 1];
                    chunk = chunk.slice(0, -1);
                }
                if (chunk) {
                    const value = this.#handle.encode(chunk);
                    if (value.length) {
                        controller.enqueue(value);
                    }
                }
            },
            flush: (controller) => {
                if (this.#pendingHighSurrogate !== null) {
                    controller.enqueue(this.#handle.encode(this.#pendingHighSurrogate));
                    this.#pendingHighSurrogate = null;
                }
            },
        });
    }
    get encoding() {
        return this.#handle.encoding;
    }
    get readable() {
        return this.#transform.readable;
    }
    get writable() {
        return this.#transform.writable;
    }
}
