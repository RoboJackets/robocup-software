import type { ReadableBoxedValues } from "svelte-toolbelt";
interface PresenceManagerOpts extends ReadableBoxedValues<{
    open: boolean;
    ref: HTMLElement | null;
}> {
    onComplete?: () => void;
    enabled?: boolean;
}
export declare class PresenceManager {
    #private;
    constructor(opts: PresenceManagerOpts);
    get shouldRender(): boolean;
}
export {};
