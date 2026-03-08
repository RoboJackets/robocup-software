export declare class Wtf8EncoderStream {
    #private;
    constructor();
    get encoding(): string;
    get readable(): ReadableStream<Uint8Array>;
    get writable(): WritableStream<string>;
}
