import { EMPTY, WTF8 } from './const.js';
function utf8length(str) {
    let len = 0;
    for (const s of str) {
        const cp = s.codePointAt(0);
        if (cp < 0x80) {
            len++;
        }
        else if (cp < 0x800) {
            len += 2;
        }
        else if (cp < 0x10000) {
            len += 3;
        }
        else {
            len += 4;
        }
    }
    return len;
}
export class Wtf8Encoder {
    encoding = WTF8;
    encode(input) {
        if (!input) {
            return EMPTY;
        }
        const buf = new Uint8Array(utf8length(String(input)));
        this.encodeInto(input, buf);
        return buf;
    }
    encodeInto(source, destination) {
        const str = String(source);
        const len = str.length;
        const outLen = destination.length;
        let written = 0;
        let read = 0;
        for (read = 0; read < len; read++) {
            const c = str.codePointAt(read);
            if (c < 0x80) {
                if (written >= outLen) {
                    break;
                }
                destination[written++] = c;
            }
            else if (c < 0x800) {
                if (written >= outLen - 1) {
                    break;
                }
                destination[written++] = 0xc0 | (c >> 6);
                destination[written++] = 0x80 | (c & 0x3f);
            }
            else if (c < 0x10000) {
                if (written >= outLen - 2) {
                    break;
                }
                destination[written++] = 0xe0 | (c >> 12);
                destination[written++] = 0x80 | ((c >> 6) & 0x3f);
                destination[written++] = 0x80 | (c & 0x3f);
            }
            else {
                if (written >= outLen - 3) {
                    break;
                }
                destination[written++] = 0xf0 | (c >> 18);
                destination[written++] = 0x80 | ((c >> 12) & 0x3f);
                destination[written++] = 0x80 | ((c >> 6) & 0x3f);
                destination[written++] = 0x80 | (c & 0x3f);
                read++;
            }
        }
        return {
            read,
            written,
        };
    }
}
