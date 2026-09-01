/** Generic observation / action. Domain types stay in adapters. */
export type Json = null | boolean | number | string | Json[] | { readonly [k: string]: Json };

export const CLAIM_TYPES = ["measured", "search_unreached", "proven", "hypothesis", "deprecated"] as const;
export type ClaimType = (typeof CLAIM_TYPES)[number];
