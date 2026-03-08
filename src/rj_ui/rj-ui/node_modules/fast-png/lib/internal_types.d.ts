export declare const ColorType: {
    readonly UNKNOWN: -1;
    readonly GREYSCALE: 0;
    readonly TRUECOLOUR: 2;
    readonly INDEXED_COLOUR: 3;
    readonly GREYSCALE_ALPHA: 4;
    readonly TRUECOLOUR_ALPHA: 6;
};
export type ColorType = (typeof ColorType)[keyof typeof ColorType];
export declare const CompressionMethod: {
    readonly UNKNOWN: -1;
    readonly DEFLATE: 0;
};
export type CompressionMethod = (typeof CompressionMethod)[keyof typeof CompressionMethod];
export declare const FilterMethod: {
    readonly UNKNOWN: -1;
    readonly ADAPTIVE: 0;
};
export type FilterMethod = (typeof FilterMethod)[keyof typeof FilterMethod];
export declare const InterlaceMethod: {
    readonly UNKNOWN: -1;
    readonly NO_INTERLACE: 0;
    readonly ADAM7: 1;
};
export type InterlaceMethod = (typeof InterlaceMethod)[keyof typeof InterlaceMethod];
export declare const DisposeOpType: {
    readonly NONE: 0;
    readonly BACKGROUND: 1;
    readonly PREVIOUS: 2;
};
export type DisposeOpType = (typeof DisposeOpType)[keyof typeof DisposeOpType];
export declare const BlendOpType: {
    readonly SOURCE: 0;
    readonly OVER: 1;
};
export type BlendOpType = (typeof BlendOpType)[keyof typeof BlendOpType];
//# sourceMappingURL=internal_types.d.ts.map