	for %%T in (*.ply) do (
			"C:\Program Files\VCG\MeshLab\meshlabserver.exe" -i %%T -o %%T -s sampleandclean.mlx
	)
