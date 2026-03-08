import type { PngDataArray } from '../types.ts';
export declare function unfilterNone(currentLine: PngDataArray, newLine: PngDataArray, bytesPerLine: number): void;
export declare function unfilterSub(currentLine: PngDataArray, newLine: PngDataArray, bytesPerLine: number, bytesPerPixel: number): void;
export declare function unfilterUp(currentLine: PngDataArray, newLine: PngDataArray, prevLine: PngDataArray, bytesPerLine: number): void;
export declare function unfilterAverage(currentLine: PngDataArray, newLine: PngDataArray, prevLine: PngDataArray, bytesPerLine: number, bytesPerPixel: number): void;
export declare function unfilterPaeth(currentLine: PngDataArray, newLine: PngDataArray, prevLine: PngDataArray, bytesPerLine: number, bytesPerPixel: number): void;
//# sourceMappingURL=unfilter.d.ts.map