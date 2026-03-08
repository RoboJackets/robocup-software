import type { IOBuffer } from 'iobuffer';
export declare const textChunkName = "tEXt";
export declare function decodetEXt(text: Record<string, string>, buffer: IOBuffer, length: number): void;
export declare function encodetEXt(buffer: IOBuffer, keyword: string, text: string): void;
export declare function readKeyword(buffer: IOBuffer): string;
export declare function readLatin1(buffer: IOBuffer, length: number): string;
//# sourceMappingURL=text.d.ts.map