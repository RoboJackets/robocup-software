export interface Wtf8DecodeOptions extends TextDecoderOptions {
    bufferSize?: number;
}
export declare class Wtf8Decoder implements TextDecoderCommon {
    #private;
    static DEFAULT_BUFFERSIZE: number;
    readonly encoding = "wtf-8";
    readonly fatal: boolean;
    readonly ignoreBOM: boolean;
    readonly bufferSize: number;
    constructor(label?: string, options?: Wtf8DecodeOptions | undefined);
    decode(input?: AllowSharedBufferSource, options?: TextDecodeOptions): string;
}
