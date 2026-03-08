import { Wtf8Decoder } from './decode.js';
export class Wtf8DecoderStream {
    #handle;
    #transform;
    constructor(encoding = 'wtf-8', options = undefined) {
        const dec = new Wtf8Decoder(encoding, options);
        this.#handle = dec;
        this.#transform = new TransformStream({
            transform(chunk, controller) {
                const value = dec.decode(chunk, { stream: true });
                if (value) {
                    controller.enqueue(value);
                }
            },
            flush(controller) {
                const value = dec.decode();
                if (value) {
                    controller.enqueue(value);
                }
                controller.terminate();
            },
        });
    }
    get encoding() {
        return this.#handle.encoding;
    }
    get fatal() {
        return this.#handle.fatal;
    }
    get ignoreBOM() {
        return this.#handle.ignoreBOM;
    }
    get readable() {
        return this.#transform.readable;
    }
    get writable() {
        return this.#transform.writable;
    }
}
