import { SYMS } from './constants.js';

interface OriginalEncoding {
    [SYMS.ENCODED]?: Uint8Array;
    [SYMS.LENGTH]?: Uint8Array;
}
interface ValueOf<T> extends OriginalEncoding {
    valueOf(): T;
}
/**
 * Get the encoded version of the object, if it has been stored on the object.
 *
 * @param obj Object to check.
 * @returns Encoded version as bytes, if available.
 */
declare function getEncoded(obj: unknown): Uint8Array | undefined;
/**
 * Get the encoded version of the length of the object or array, if it has
 * been stored.
 *
 * @param obj Object to check.
 * @returns Encoded length as bytes, if available.
 */
declare function getEncodedLength(obj: unknown): Uint8Array | undefined;
/**
 * Save the original encoding of the given object on the oject as a property
 * with a Symbol name, so it can be later extracted for round-tripping or
 * crypto.
 *
 * @param obj Object to tag.
 * @param orig Originally-encoded version of the object.
 */
declare function saveEncoded(obj: OriginalEncoding, orig: Uint8Array): void;
/**
 * Save the original encoding for the length of an object or array, so that
 * it can be later extracted.  Include the major type in the original bytes.
 *
 * @param obj Object to store a length on.
 * @param orig Originally-encoded version of the length, including major type.
 */
declare function saveEncodedLength(obj: OriginalEncoding, orig: Uint8Array): void;
declare function box(value: bigint, orig: Uint8Array): BigInt;
declare function box(value: string, orig: Uint8Array): String;
declare function box(value: number, orig: Uint8Array): Number;
declare function box(value: bigint | number, orig: Uint8Array): BigInt | Number;
declare function box(value: bigint | number | string, orig: Uint8Array): BigInt | Number | String;
/**
 * Remove all boxed types from an object.
 *
 * @param obj Object ot unbox.
 * @returns Unboxed copy.
 */
declare function unbox(obj: unknown): unknown;

export { type OriginalEncoding, type ValueOf, box, getEncoded, getEncodedLength, saveEncoded, saveEncodedLength, unbox };
