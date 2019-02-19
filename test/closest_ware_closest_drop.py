import baseline_greedy_closest_wares.closest_ware_closest_drop as cwcd
print("Starting evaluation")
cwcd.evaluate(render=True, robots=20, spawn=100, shelve_length=5, shelve_width=5, shelve_height=5, steps=1000)


