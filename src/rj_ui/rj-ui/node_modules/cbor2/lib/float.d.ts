/**
 * Parse a big endian float16 from a buffer.
 *
 * @param buf Buffer to read from.
 * @param offset Offset into buf to start reading 2 octets.
 * @param rejectSubnormals Throw if the result is subnormal.
 * @returns Parsed float.
 * @throws Unwanted subnormal.
 */
declare function parseHalf(buf: Uint8Array, offset?: number, rejectSubnormals?: boolean): number;
/**
 * Return a big-endian unsigned integer that has the same internal layout
 * as the given number as a float16, if it fits.  Otherwise returns null.
 *
 * @param half The number to convert to a half-precision float.  Must fit into
 *   at least a float32.
 * @returns Number on success, otherwise null.  Make sure to check with
 *   `=== null`, in case this returns 0, which is valid.
 */
declare function halfToUint(half: number): number | null;
/**
 * Flush subnormal numbers to 0/-0.
 *
 * @param n Number.
 * @returns Normalized number.
 */
declare function flushToZero(n: number): number;
/**
 * Does the given buffer contain a bigEndian IEEE754 float that is subnormal?
 * If so, throw an error.
 *
 * @param buf 2, 4, or 8 bytes for float16, float32, or float64.
 * @throws Bad input or subnormal.
 */
declare function checkSubnormal(buf: Uint8Array): void;

export { checkSubnormal, flushToZero, halfToUint, parseHalf };
