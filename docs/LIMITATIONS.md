# Known Limitations & Future Work

## Current Limitations

1. **Scalability**: Practical for up to ~10 robots, ~20 tasks
2. **No task dependencies**: Tasks must be independent
3. **Offline planning only**: No real-time replanning
4. **Simplified spatial model**: All locations can be [0,0]
5. **Fixed task durations**: No uncertainty or WCET bounds
6. **Single objective**: Only minimizes makespan

## Future Work

- Dynamic replanning with incremental SMT
- Task precedence and dependencies
- Multi-objective optimization (energy, fairness)
- Real-world validation on robot fleets
- Integration with ROS 2 executor

See paper for detailed roadmap.
