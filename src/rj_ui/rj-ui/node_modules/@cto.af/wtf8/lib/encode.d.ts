export declare class Wtf8Encoder implements TextEncoderCommon {
    readonly encoding = "wtf-8";
    encode(input?: string): Uint8Array;
    encodeInto(source: string, destination: Uint8Array): TextEncoderEncodeIntoResult;
}
