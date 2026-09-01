# Inventory domain

Partial-observation reorder lab used to test AgentArena — not a solved inventory optimizer.

```bash
npm run inventory:smoke
```

Gates: fill rate ≥ 0.95, cash > 0, stockout days < 5, zero constraint violations, supplier-alert delay < 3. Baseline **FAIL**.
