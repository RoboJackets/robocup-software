export declare class Wtf8DecoderStream {
    #private;
    constructor(encoding?: string, options?: TextDecoderOptions | undefined);
    get encoding(): string;
    get fatal(): boolean;
    get ignoreBOM(): boolean;
    get readable(): ReadableStream<string>;
    get writable(): WritableStream<AllowSharedBufferSource>;
}
