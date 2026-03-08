export declare class DecodeError extends TypeError {
    code: string;
    constructor();
}
export declare class InvalidEncodingError extends RangeError {
    code: string;
    constructor(label: string);
}
