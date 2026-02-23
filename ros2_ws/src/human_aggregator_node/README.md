## human_aggregator_node

Aggregates multiple Unity-published human topics (e.g., /human1, /human2, …) of type unitycustommsg/msg/TwistTransformUnity into a single /all_humans stream (unitycustommsg/msg/HumanArray).
Useful for Nav2 layers, evaluation nodes, or logging—so downstream consumers subscribe once.

What it does

* Auto-discovers topics named /human* publishing TwistTransformUnity.
* Subscribes to each and stores the latest message.
* Re-publishes a HumanArray at a fixed rate.

**Publishes**

* /all_humans (unitycustommsg/HumanArray)

**Subscribes (auto)**

* /human* (unitycustommsg/TwistTransformUnity) — any number of topics, e.g., /human1, /human2, …