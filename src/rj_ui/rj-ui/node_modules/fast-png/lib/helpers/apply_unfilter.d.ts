/**
 * Apllies filter on scanline based on the filter type.
 * @param filterType - The filter type to apply.
 * @param currentLine - The current line of pixel data.
 * @param newLine - The new line of pixel data.
 * @param prevLine - The previous line of pixel data.
 * @param passLineBytes - The number of bytes in the pass line.
 * @param bytesPerPixel - The number of bytes per pixel.
 */
export declare function applyUnfilter(filterType: number, currentLine: Uint8Array, newLine: Uint8Array, prevLine: Uint8Array, passLineBytes: number, bytesPerPixel: number): void;
//# sourceMappingURL=apply_unfilter.d.ts.map