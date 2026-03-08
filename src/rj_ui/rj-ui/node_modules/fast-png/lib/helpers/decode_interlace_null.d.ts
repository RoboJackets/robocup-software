import type { PngDataArray } from '../types.ts';
export interface DecodeInterlaceNullParams {
    data: Uint8Array;
    width: number;
    height: number;
    channels: number;
    depth: number;
}
export declare function decodeInterlaceNull(params: DecodeInterlaceNullParams): PngDataArray;
//# sourceMappingURL=decode_interlace_null.d.ts.map