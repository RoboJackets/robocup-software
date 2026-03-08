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
export type ServerCogProps = typeof __propDef.props;
export type ServerCogEvents = typeof __propDef.events;
export type ServerCogSlots = typeof __propDef.slots;
/**
 * @component @name ServerCog
 * @description Lucide SVG icon component, renders SVG Element with children.
 *
 * @preview ![img](data:image/svg+xml;base64,PHN2ZyAgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIgogIHdpZHRoPSIyNCIKICBoZWlnaHQ9IjI0IgogIHZpZXdCb3g9IjAgMCAyNCAyNCIKICBmaWxsPSJub25lIgogIHN0cm9rZT0iIzAwMCIgc3R5bGU9ImJhY2tncm91bmQtY29sb3I6ICNmZmY7IGJvcmRlci1yYWRpdXM6IDJweCIKICBzdHJva2Utd2lkdGg9IjIiCiAgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIgogIHN0cm9rZS1saW5lam9pbj0icm91bmQiCj4KICA8cGF0aCBkPSJtMTAuODUyIDE0Ljc3Mi0uMzgzLjkyMyIgLz4KICA8cGF0aCBkPSJNMTMuMTQ4IDE0Ljc3MmEzIDMgMCAxIDAtMi4yOTYtNS41NDRsLS4zODMtLjkyMyIgLz4KICA8cGF0aCBkPSJtMTMuMTQ4IDkuMjI4LjM4My0uOTIzIiAvPgogIDxwYXRoIGQ9Im0xMy41MyAxNS42OTYtLjM4Mi0uOTI0YTMgMyAwIDEgMS0yLjI5Ni01LjU0NCIgLz4KICA8cGF0aCBkPSJtMTQuNzcyIDEwLjg1Mi45MjMtLjM4MyIgLz4KICA8cGF0aCBkPSJtMTQuNzcyIDEzLjE0OC45MjMuMzgzIiAvPgogIDxwYXRoIGQ9Ik00LjUgMTBINGEyIDIgMCAwIDEtMi0yVjRhMiAyIDAgMCAxIDItMmgxNmEyIDIgMCAwIDEgMiAydjRhMiAyIDAgMCAxLTIgMmgtLjUiIC8+CiAgPHBhdGggZD0iTTQuNSAxNEg0YTIgMiAwIDAgMC0yIDJ2NGEyIDIgMCAwIDAgMiAyaDE2YTIgMiAwIDAgMCAyLTJ2LTRhMiAyIDAgMCAwLTItMmgtLjUiIC8+CiAgPHBhdGggZD0iTTYgMThoLjAxIiAvPgogIDxwYXRoIGQ9Ik02IDZoLjAxIiAvPgogIDxwYXRoIGQ9Im05LjIyOCAxMC44NTItLjkyMy0uMzgzIiAvPgogIDxwYXRoIGQ9Im05LjIyOCAxMy4xNDgtLjkyMy4zODMiIC8+Cjwvc3ZnPgo=) - https://lucide.dev/icons/server-cog
 * @see https://lucide.dev/guide/packages/lucide-svelte - Documentation
 *
 * @param {Object} props - Lucide icons props and any valid SVG attribute
 * @returns {FunctionalComponent} Svelte component
 *
 */
export default class ServerCog extends SvelteComponentTyped<ServerCogProps, ServerCogEvents, ServerCogSlots> {
}
export {};
