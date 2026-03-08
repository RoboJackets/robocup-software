import { type Getter } from "svelte-toolbelt";
export interface SafePolygonOptions {
    enabled: Getter<boolean>;
    triggerNode: Getter<HTMLElement | null>;
    contentNode: Getter<HTMLElement | null>;
    onPointerExit: () => void;
    buffer?: number;
}
/**
 * Creates a safe polygon area that allows users to move their cursor between
 * the trigger and floating content without closing it.
 */
export declare class SafePolygon {
    #private;
    constructor(opts: SafePolygonOptions);
}
