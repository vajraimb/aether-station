#!/usr/bin/env npx tsx
/**
 * Inventory domain smoke. Platform check, not a solved policy.
 *   npm run inventory:smoke
 * Writes outputs/inventory/smoke.json (tracked) and per-scenario traces (gitignored).
 */
import { runInventorySmoke } from "./smoke.ts";

runInventorySmoke(process.argv.includes("--smoke") || process.argv.includes("smoke") ? 30 : 60);
