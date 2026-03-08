import { BOM, EMPTY, MIN_HIGH_SURROGATE, MIN_LOW_SURROGATE, REPLACEMENT, WTF8, } from './const.js';
import { DecodeError, InvalidEncodingError } from './errors.js';
function isArrayBufferView(input) {
    return (input &&
        !(input instanceof ArrayBuffer) &&
        input.buffer instanceof ArrayBuffer);
}
function getUint8(input) {
    if (!input) {
        return EMPTY;
    }
    if (input instanceof Uint8Array) {
        return input;
    }
    if (isArrayBufferView(input)) {
        return new Uint8Array(input.buffer, input.byteOffset, input.byteLength);
    }
    return new Uint8Array(input);
}
const REMAINDER = [
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    -1,
    -1,
    -1,
    -1,
    1,
    1,
    2,
    3,
];
export class Wtf8Decoder {
    static DEFAULT_BUFFERSIZE = 0x1000;
    encoding = WTF8;
    fatal;
    ignoreBOM;
    bufferSize;
    #left = 0;
    #cur = 0;
    #pending = 0;
    #first = true;
    #buf;
    constructor(label = 'wtf8', options = undefined) {
        if (label.toLowerCase().replace('-', '') !== 'wtf8') {
            throw new InvalidEncodingError(label);
        }
        this.fatal = Boolean(options?.fatal);
        this.ignoreBOM = Boolean(options?.ignoreBOM);
        this.bufferSize = Math.floor(options?.bufferSize ?? Wtf8Decoder.DEFAULT_BUFFERSIZE);
        if (isNaN(this.bufferSize) || (this.bufferSize < 1)) {
            throw new RangeError(`Invalid buffer size: ${options?.bufferSize}`);
        }
        this.#buf = new Uint16Array(this.bufferSize);
    }
    decode(input, options) {
        const streaming = Boolean(options?.stream);
        const bytes = getUint8(input);
        const res = [];
        const out = this.#buf;
        const maxSize = this.bufferSize - 3;
        let pos = 0;
        const fatal = () => {
            this.#cur = 0;
            this.#left = 0;
            this.#pending = 0;
            if (this.fatal) {
                throw new DecodeError();
            }
            out[pos++] = REPLACEMENT;
        };
        const fatals = () => {
            const p = this.#pending;
            for (let i = 0; i < p; i++) {
                fatal();
            }
        };
        const oneByte = (b) => {
            if (this.#left === 0) {
                const n = REMAINDER[b >> 4];
                switch (n) {
                    case -1:
                        fatal();
                        break;
                    case 0:
                        out[pos++] = b;
                        break;
                    case 1:
                        this.#cur = b & 0x1f;
                        if ((this.#cur & 0x1e) === 0) {
                            fatal();
                        }
                        else {
                            this.#left = 1;
                            this.#pending = 1;
                        }
                        break;
                    case 2:
                        this.#cur = b & 0x0f;
                        this.#left = 2;
                        this.#pending = 1;
                        break;
                    case 3:
                        if (b & 0x08) {
                            fatal();
                        }
                        else {
                            this.#cur = b & 0x07;
                            this.#left = 3;
                            this.#pending = 1;
                        }
                        break;
                }
            }
            else {
                if ((b & 0xc0) !== 0x80) {
                    fatals();
                    return oneByte(b);
                }
                if ((this.#pending === 1) &&
                    (this.#left === 2) &&
                    (this.#cur === 0) &&
                    ((b & 0x20) === 0)) {
                    fatals();
                    return oneByte(b);
                }
                if ((this.#left === 3) && (this.#cur === 0) && ((b & 0x30) === 0)) {
                    fatals();
                    return oneByte(b);
                }
                this.#cur = (this.#cur << 6) | (b & 0x3f);
                this.#pending++;
                if (--this.#left === 0) {
                    if (this.ignoreBOM || !this.#first || (this.#cur !== BOM)) {
                        if (this.#cur < 0x10000) {
                            out[pos++] = this.#cur;
                        }
                        else {
                            const cp = this.#cur - 0x10000;
                            out[pos++] = ((cp >>> 10) & 0x3ff) | MIN_HIGH_SURROGATE;
                            out[pos++] = (cp & 0x3ff) | MIN_LOW_SURROGATE;
                        }
                    }
                    this.#cur = 0;
                    this.#pending = 0;
                    this.#first = false;
                }
            }
        };
        for (const b of bytes) {
            if (pos >= maxSize) {
                res.push(String.fromCharCode.apply(null, out.subarray(0, pos)));
                pos = 0;
            }
            oneByte(b);
        }
        if (!streaming) {
            this.#first = true;
            if (this.#cur || this.#left) {
                fatals();
            }
        }
        if (pos > 0) {
            res.push(String.fromCharCode.apply(null, out.subarray(0, pos)));
        }
        return res.join('');
    }
}
