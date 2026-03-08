/**
 * Key, value, and key CBOR-encoded with the current options.  When used for
 * sorting, ONLY the keyEncoded element is used.  The value elements is
 * retained so that the sorted version can be encoded later.
 */
type KeyValueEncoded = [
    keyDecoded: unknown,
    value: unknown,
    keyEncoded: Uint8Array
];
/**
 * Sort keys in an object or Map before encoding.  Only the first element of the
 * array can be used for sorting.
 */
type KeySorter = (a: KeyValueEncoded, b: KeyValueEncoded) => number;
/**
 * Sort according to RFC 8949, section 4.2.1
 * (https://www.rfc-editor.org/rfc/rfc8949.html#name-core-deterministic-encoding).
 *
 * @param a First item.
 * @param b Second item.
 * @returns Negative for a < b, Positive for b > a, 0 if equal.
 */
declare function sortCoreDeterministic(a: KeyValueEncoded, b: KeyValueEncoded): number;
/**
 * Sort according to RFC 8949, section 4.2.3
 * (https://www.rfc-editor.org/rfc/rfc8949.html#name-length-first-map-key-orderi).
 *
 * @param a First item.
 * @param b Second item.
 * @returns Negative for a < b, Positive for b > a, 0 if equal.
 */
declare function sortLengthFirstDeterministic(a: KeyValueEncoded, b: KeyValueEncoded): number;

export { type KeySorter, type KeyValueEncoded, sortCoreDeterministic, sortLengthFirstDeterministic };
