import { afterTick, onDestroyEffect } from "svelte-toolbelt";
export class AnimationsComplete {
    #opts;
    #currentFrame = null;
    constructor(opts) {
        this.#opts = opts;
        onDestroyEffect(() => this.#cleanup());
    }
    #cleanup() {
        if (!this.#currentFrame)
            return;
        window.cancelAnimationFrame(this.#currentFrame);
        this.#currentFrame = null;
    }
    run(fn) {
        // if already running, cleanup and restart
        this.#cleanup();
        const node = this.#opts.ref.current;
        if (!node)
            return;
        if (typeof node.getAnimations !== "function") {
            this.#executeCallback(fn);
            return;
        }
        this.#currentFrame = window.requestAnimationFrame(() => {
            const animations = node.getAnimations();
            if (animations.length === 0) {
                this.#executeCallback(fn);
                return;
            }
            Promise.allSettled(animations.map((animation) => animation.finished)).then(() => {
                this.#executeCallback(fn);
            });
        });
    }
    #executeCallback(fn) {
        const execute = () => {
            fn();
        };
        if (this.#opts.afterTick) {
            afterTick(execute);
        }
        else {
            execute();
        }
    }
}
