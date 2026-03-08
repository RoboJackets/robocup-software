import { IOBuffer } from 'iobuffer';
import { deflate } from 'pako';
import { writeCrc } from "./helpers/crc.js";
import { writeSignature } from "./helpers/signature.js";
import { encodetEXt } from "./helpers/text.js";
import { ColorType, CompressionMethod, FilterMethod, InterlaceMethod, } from "./internal_types.js";
const defaultZlibOptions = {
    level: 3,
};
export default class PngEncoder extends IOBuffer {
    _png;
    _zlibOptions;
    _colorType;
    _interlaceMethod;
    constructor(data, options = {}) {
        super();
        this._colorType = ColorType.UNKNOWN;
        this._zlibOptions = { ...defaultZlibOptions, ...options.zlib };
        this._png = this._checkData(data);
        this._interlaceMethod =
            (options.interlace === 'Adam7'
                ? InterlaceMethod.ADAM7
                : InterlaceMethod.NO_INTERLACE) ?? InterlaceMethod.NO_INTERLACE;
        this.setBigEndian();
    }
    encode() {
        writeSignature(this);
        this.encodeIHDR();
        if (this._png.palette) {
            this.encodePLTE();
            if (this._png.palette[0].length === 4) {
                this.encodeTRNS();
            }
        }
        this.encodeData();
        if (this._png.text) {
            for (const [keyword, text] of Object.entries(this._png.text)) {
                encodetEXt(this, keyword, text);
            }
        }
        this.encodeIEND();
        return this.toArray();
    }
    // https://www.w3.org/TR/PNG/#11IHDR
    encodeIHDR() {
        this.writeUint32(13);
        this.writeChars('IHDR');
        this.writeUint32(this._png.width);
        this.writeUint32(this._png.height);
        this.writeByte(this._png.depth);
        this.writeByte(this._colorType);
        this.writeByte(CompressionMethod.DEFLATE);
        this.writeByte(FilterMethod.ADAPTIVE);
        this.writeByte(this._interlaceMethod);
        writeCrc(this, 17);
    }
    // https://www.w3.org/TR/PNG/#11IEND
    encodeIEND() {
        this.writeUint32(0);
        this.writeChars('IEND');
        writeCrc(this, 4);
    }
    encodePLTE() {
        const paletteLength = this._png.palette?.length * 3;
        this.writeUint32(paletteLength);
        this.writeChars('PLTE');
        for (const color of this._png.palette) {
            this.writeByte(color[0]);
            this.writeByte(color[1]);
            this.writeByte(color[2]);
        }
        writeCrc(this, 4 + paletteLength);
    }
    encodeTRNS() {
        const alpha = this._png.palette.filter((color) => {
            return color.at(-1) !== 255;
        });
        this.writeUint32(alpha.length);
        this.writeChars('tRNS');
        for (const el of alpha) {
            this.writeByte(el.at(-1));
        }
        writeCrc(this, 4 + alpha.length);
    }
    // https://www.w3.org/TR/PNG/#11IDAT
    encodeIDAT(data) {
        this.writeUint32(data.length);
        this.writeChars('IDAT');
        this.writeBytes(data);
        writeCrc(this, data.length + 4);
    }
    encodeData() {
        const { width, height, channels, depth, data } = this._png;
        const slotsPerLine = depth <= 8
            ? Math.ceil((width * depth) / 8) * channels
            : Math.ceil((((width * depth) / 8) * channels) / 2);
        const newData = new IOBuffer().setBigEndian();
        let offset = 0;
        if (this._interlaceMethod === InterlaceMethod.NO_INTERLACE) {
            for (let i = 0; i < height; i++) {
                newData.writeByte(0); // no filter
                if (depth === 16) {
                    offset = writeDataUint16(data, newData, slotsPerLine, offset);
                }
                else {
                    offset = writeDataBytes(data, newData, slotsPerLine, offset);
                }
            }
        }
        else if (this._interlaceMethod === InterlaceMethod.ADAM7) {
            // Adam7 interlacing
            offset = writeDataInterlaced(this._png, data, newData, offset);
        }
        const buffer = newData.toArray();
        const compressed = deflate(buffer, this._zlibOptions);
        this.encodeIDAT(compressed);
    }
    _checkData(data) {
        const { colorType, channels, depth } = getColorType(data, data.palette);
        const png = {
            width: checkInteger(data.width, 'width'),
            height: checkInteger(data.height, 'height'),
            channels,
            data: data.data,
            depth,
            text: data.text,
            palette: data.palette,
        };
        this._colorType = colorType;
        const expectedSize = depth < 8
            ? Math.ceil((png.width * depth) / 8) * png.height * channels
            : png.width * png.height * channels;
        if (png.data.length !== expectedSize) {
            throw new RangeError(`wrong data size. Found ${png.data.length}, expected ${expectedSize}`);
        }
        return png;
    }
}
function checkInteger(value, name) {
    if (Number.isInteger(value) && value > 0) {
        return value;
    }
    throw new TypeError(`${name} must be a positive integer`);
}
function getColorType(data, palette) {
    const { channels = 4, depth = 8 } = data;
    if (channels !== 4 && channels !== 3 && channels !== 2 && channels !== 1) {
        throw new RangeError(`unsupported number of channels: ${channels}`);
    }
    const returnValue = {
        channels,
        depth,
        colorType: ColorType.UNKNOWN,
    };
    switch (channels) {
        case 4:
            returnValue.colorType = ColorType.TRUECOLOUR_ALPHA;
            break;
        case 3:
            returnValue.colorType = ColorType.TRUECOLOUR;
            break;
        case 1:
            if (palette) {
                returnValue.colorType = ColorType.INDEXED_COLOUR;
            }
            else {
                returnValue.colorType = ColorType.GREYSCALE;
            }
            break;
        case 2:
            returnValue.colorType = ColorType.GREYSCALE_ALPHA;
            break;
        default:
            throw new Error('unsupported number of channels');
    }
    return returnValue;
}
function writeDataBytes(data, newData, slotsPerLine, offset) {
    for (let j = 0; j < slotsPerLine; j++) {
        newData.writeByte(data[offset++]);
    }
    return offset;
}
function writeDataInterlaced(imageData, data, newData, offset) {
    const passes = [
        { x: 0, y: 0, xStep: 8, yStep: 8 },
        { x: 4, y: 0, xStep: 8, yStep: 8 },
        { x: 0, y: 4, xStep: 4, yStep: 8 },
        { x: 2, y: 0, xStep: 4, yStep: 4 },
        { x: 0, y: 2, xStep: 2, yStep: 4 },
        { x: 1, y: 0, xStep: 2, yStep: 2 },
        { x: 0, y: 1, xStep: 1, yStep: 2 },
    ];
    const { width, height, channels, depth } = imageData;
    let pixelSize;
    if (depth === 16) {
        pixelSize = (channels * depth) / 8 / 2;
    }
    else {
        pixelSize = (channels * depth) / 8;
    }
    // Process each pass
    for (let passIndex = 0; passIndex < 7; passIndex++) {
        const pass = passes[passIndex];
        const passWidth = Math.floor((width - pass.x + pass.xStep - 1) / pass.xStep);
        const passHeight = Math.floor((height - pass.y + pass.yStep - 1) / pass.yStep);
        if (passWidth <= 0 || passHeight <= 0)
            continue;
        const passLineBytes = passWidth * pixelSize;
        // For each scanline in this pass
        for (let y = 0; y < passHeight; y++) {
            const imageY = pass.y + y * pass.yStep;
            // Extract raw scanline data
            const rawScanline = depth <= 8
                ? new Uint8Array(passLineBytes)
                : new Uint16Array(passLineBytes);
            let rawOffset = 0;
            for (let x = 0; x < passWidth; x++) {
                const imageX = pass.x + x * pass.xStep;
                if (imageX < width && imageY < height) {
                    const srcPos = (imageY * width + imageX) * pixelSize;
                    for (let i = 0; i < pixelSize; i++) {
                        rawScanline[rawOffset++] = data[srcPos + i];
                    }
                }
            }
            newData.writeByte(0); // no filter
            if (depth === 8) {
                newData.writeBytes(rawScanline);
            }
            else if (depth === 16) {
                for (const value of rawScanline) {
                    newData.writeByte((value >> 8) & 0xff); // High byte
                    newData.writeByte(value & 0xff);
                }
            }
        }
    }
    return offset;
}
function writeDataUint16(data, newData, slotsPerLine, offset) {
    for (let j = 0; j < slotsPerLine; j++) {
        newData.writeUint16(data[offset++]);
    }
    return offset;
}
//# sourceMappingURL=png_encoder.js.map