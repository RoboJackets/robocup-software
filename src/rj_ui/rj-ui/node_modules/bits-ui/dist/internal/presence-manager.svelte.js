import { watch } from "runed";
import { AnimationsComplete } from "./animations-complete.js";
export class PresenceManager {
    #opts;
    #enabled;
    #afterAnimations;
    #shouldRender = $state(false);
    constructor(opts) {
        this.#opts = opts;
        this.#shouldRender = opts.open.current;
        this.#enabled = opts.enabled ?? true;
        this.#afterAnimations = new AnimationsComplete({
            ref: this.#opts.ref,
            afterTick: this.#opts.open,
        });
        watch(() => this.#opts.open.current, (isOpen) => {
            if (isOpen)
                this.#shouldRender = true;
            if (!this.#enabled)
                return;
            this.#afterAnimations.run(() => {
                if (isOpen === this.#opts.open.current) {
                    if (!this.#opts.open.current) {
                        this.#shouldRender = false;
                    }
                    this.#opts.onComplete?.();
                }
            });
        });
    }
    get shouldRender() {
        return this.#shouldRender;
    }
}
