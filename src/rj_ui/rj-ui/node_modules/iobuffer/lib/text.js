/**
 * Decode bytes to text
 * @param bytes - Bytes to decode
 * @param encoding - Text encoding
 * @returns The decoded text
 */
export function decode(bytes, encoding = 'utf8') {
    const decoder = new TextDecoder(encoding);
    return decoder.decode(bytes);
}
const encoder = new TextEncoder();
/**
 * Encode text to utf8
 * @param str - Text to encode
 * @returns The encoded bytes
 */
export function encode(str) {
    return encoder.encode(str);
}
//# sourceMappingURL=text.js.map