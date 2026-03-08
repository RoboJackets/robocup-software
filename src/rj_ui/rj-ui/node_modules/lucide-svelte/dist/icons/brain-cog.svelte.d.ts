/**
 * @license lucide-svelte v0.562.0 - ISC
 *
 * ISC License
 * 
 * Copyright (c) for portions of Lucide are held by Cole Bemis 2013-2023 as part of Feather (MIT). All other copyright (c) for Lucide are held by Lucide Contributors 2025.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * 
 * ---
 * 
 * The MIT License (MIT) (for portions derived from Feather)
 * 
 * Copyright (c) 2013-2023 Cole Bemis
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */
import { SvelteComponentTyped } from "svelte";
import type { IconProps } from '../types.js';
declare const __propDef: {
    props: IconProps;
    events: {
        [evt: string]: CustomEvent<any>;
    };
    slots: {
        default: {};
    };
};
export type BrainCogProps = typeof __propDef.props;
export type BrainCogEvents = typeof __propDef.events;
export type BrainCogSlots = typeof __propDef.slots;
/**
 * @component @name BrainCog
 * @description Lucide SVG icon component, renders SVG Element with children.
 *
 * @preview ![img](data:image/svg+xml;base64,PHN2ZyAgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIgogIHdpZHRoPSIyNCIKICBoZWlnaHQ9IjI0IgogIHZpZXdCb3g9IjAgMCAyNCAyNCIKICBmaWxsPSJub25lIgogIHN0cm9rZT0iIzAwMCIgc3R5bGU9ImJhY2tncm91bmQtY29sb3I6ICNmZmY7IGJvcmRlci1yYWRpdXM6IDJweCIKICBzdHJva2Utd2lkdGg9IjIiCiAgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIgogIHN0cm9rZS1saW5lam9pbj0icm91bmQiCj4KICA8cGF0aCBkPSJtMTAuODUyIDE0Ljc3Mi0uMzgzLjkyMyIgLz4KICA8cGF0aCBkPSJtMTAuODUyIDkuMjI4LS4zODMtLjkyMyIgLz4KICA8cGF0aCBkPSJtMTMuMTQ4IDE0Ljc3Mi4zODIuOTI0IiAvPgogIDxwYXRoIGQ9Im0xMy41MzEgOC4zMDUtLjM4My45MjMiIC8+CiAgPHBhdGggZD0ibTE0Ljc3MiAxMC44NTIuOTIzLS4zODMiIC8+CiAgPHBhdGggZD0ibTE0Ljc3MiAxMy4xNDguOTIzLjM4MyIgLz4KICA8cGF0aCBkPSJNMTcuNTk4IDYuNUEzIDMgMCAxIDAgMTIgNWEzIDMgMCAwIDAtNS42My0xLjQ0NiAzIDMgMCAwIDAtLjM2OCAxLjU3MSA0IDQgMCAwIDAtMi41MjUgNS43NzEiIC8+CiAgPHBhdGggZD0iTTE3Ljk5OCA1LjEyNWE0IDQgMCAwIDEgMi41MjUgNS43NzEiIC8+CiAgPHBhdGggZD0iTTE5LjUwNSAxMC4yOTRhNCA0IDAgMCAxLTEuNSA3LjcwNiIgLz4KICA8cGF0aCBkPSJNNC4wMzIgMTcuNDgzQTQgNCAwIDAgMCAxMS40NjQgMjBjLjE4LS4zMTEuODkyLS4zMTEgMS4wNzIgMGE0IDQgMCAwIDAgNy40MzItMi41MTYiIC8+CiAgPHBhdGggZD0iTTQuNSAxMC4yOTFBNCA0IDAgMCAwIDYgMTgiIC8+CiAgPHBhdGggZD0iTTYuMDAyIDUuMTI1YTMgMyAwIDAgMCAuNCAxLjM3NSIgLz4KICA8cGF0aCBkPSJtOS4yMjggMTAuODUyLS45MjMtLjM4MyIgLz4KICA8cGF0aCBkPSJtOS4yMjggMTMuMTQ4LS45MjMuMzgzIiAvPgogIDxjaXJjbGUgY3g9IjEyIiBjeT0iMTIiIHI9IjMiIC8+Cjwvc3ZnPgo=) - https://lucide.dev/icons/brain-cog
 * @see https://lucide.dev/guide/packages/lucide-svelte - Documentation
 *
 * @param {Object} props - Lucide icons props and any valid SVG attribute
 * @returns {FunctionalComponent} Svelte component
 *
 */
export default class BrainCog extends SvelteComponentTyped<BrainCogProps, BrainCogEvents, BrainCogSlots> {
}
export {};
