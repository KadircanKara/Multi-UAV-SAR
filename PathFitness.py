def ws_fitness_evaluation(algorithm):
    # Use the custom weighted sum (normalized objective) for fitness evaluation
    return algorithm.pop.get("weighted_F")