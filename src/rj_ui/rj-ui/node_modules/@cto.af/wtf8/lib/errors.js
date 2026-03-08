export class DecodeError extends TypeError {
    code = 'ERR_ENCODING_INVALID_ENCODED_DATA';
    constructor() {
        super('The encoded data was not valid for encoding wtf-8');
    }
}
export class InvalidEncodingError extends RangeError {
    code = 'ERR_ENCODING_NOT_SUPPORTED';
    constructor(label) {
        super(`Invalid encoding: "${label}"`);
    }
}
