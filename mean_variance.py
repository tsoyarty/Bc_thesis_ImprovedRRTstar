import numpy as np

def calculate_mean_and_variance(times):
    mean = np.mean(times)
    variance = np.var(times)
    return mean, variance

#first
first_times_RRTstarML = [0.29816, 0.579933, 0.394089, 0.479521, 0.208923, 0.028908, 0.20223, 0.284464, 0.166699, 0.143014]
first_times_RRTstar = [0.00588506, 0.00634701, 0.0058112, 0.00621462, 0.00575138, 0.00715336, 0.00564742, 0.0107022, 0.00611433, 0.00555456]
first_times_InformedRRTstar =[0.00665051, 0.00657216, 0.00716292, 0.00740618, 0.00838696, 0.0072491, 0.00718217, 0.00813744, 0.00729666, 0.00726429]
first_times_RRTsharp = [0.0105573, 0.0107536, 0.0106861, 0.0092244, 0.0113239, 0.0183526, 0.0162892, 0.0107971, 0.0112971, 0.0098909]
first_times_RRTXstatic = [0.0098177, 0.0117654, 0.0113814, 0.0112381, 0.0116839, 0.0112521, 0.0192466, 0.0100667, 0.0107015, 0.0105984]
# final

final_times_RRTstarML = []
final_times_RRTstar = [0.300722, 0.35132, 0.330617, 0.341312, 0.320586, 0.321214, 0.310691, 0.313857, 0.311251, 0.311155]
final_times_InformedRRTstar =[0.260684, 0.260877, 0.26067, 0.260828, 0.257217, 0.250609, 0.260582, 0.261213, 0.257155, 0.260624]
final_times_RRTsharp = [0.30082, 0.311148, 0.321249, 0.300673, 0.321327, 0.301255, 0.291734, 0.301311, 0.290668, 0.310778]
final_times_RRTXstatic = [0.311415, 0.290695, 0.301187, 0.311544, 0.301342, 0.301363, 0.300795, 0.301651, 0.311343, 0.321259]


mean, variance = calculate_mean_and_variance(final_times_RRTstar)
print(f"Průměrná hodnota:    {mean} sekund")
print(f"Rozptyl:             {variance} (sekund)^2")
print(f"Smerodatna odchylka: {np.sqrt(variance)} (sekund)")

