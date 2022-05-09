#!/bin/bash

function runscript()
{
	make && OUTFILE=$1 SCHED=$2 LD_PRELOAD=./nvbit/mem_trace/libmem_trace.so ./rt ${@:3}
}

function simple_run()
{
	mesh=$1
	heu=$2
	order=$3
	vol=$4
	appr=$5
	pers=$6
	sched=$7
	size=$8
	tracename=COMPACT_${mesh}_${heu}_${order}_${vol}_${appr}_${pers}_${sched}_${size}.trace
	runscript $tracename keep $mesh $size ${@:9} $heu $vol $order $appr $pers $sched

	echo "<p><a href=\"https://riemann.dev/gpu-blame/?url=https://vbu.ee/kai/$tracename\">$tracename</a></p>" >> links5.html
}
function matrixfigure()
{
	simple_run $1 sah bfs aabb whilewhile persistent norm 512 ${@:2}
	simple_run $1 sah bfs aabb whilewhile persistent sm 512 ${@:2}
	simple_run $1 med bfs aabb whilewhile persistent norm 512 ${@:2}
	simple_run $1 sah random aabb whilewhile persistent norm 512 ${@:2}
	simple_run $1 sah bfs aabb ifif persistent norm 512 ${@:2}
	simple_run $1 sah bfs aabb whilewhile persistent norm 2k ${@:2}
	../scripts/upload.sh links5.html links_matrix_$1.html
}
function caches()
{
	mesh=$1
	heu=$2
	order=$3
	vol=$4
	appr=$5
	pers=$6
	sched=$7
	size=$8
	tracename=COMPACT_${mesh}_${heu}_${order}_${vol}_${appr}_${pers}_${sched}_${size}.trace

	L1=$(nv-nsight-cu-cli ./rt $mesh $size ${@:9} $heu $vol $order $appr $pers $sched | grep "L1 Hit Rate" | cut -d '%' -f2 | sed -e 's/^[[:space:]]*//')
	L2=$(nv-nsight-cu-cli ./rtNC $mesh $size ${@:9} $heu $vol $order $appr $pers $sched | grep "L2 Hit Rate" | cut -d '%' -f2 | sed -e 's/^[[:space:]]*//')
	Took=$(./rt $mesh $size ${@:9} $heu $vol $order $appr $pers $sched | grep "Took")

	echo "$tracename $L1 $L2 $Took"
}

function cachesmatrixfigure()
{
	caches $1 sah bfs aabb whilewhile persistent norm 512 ${@:2}
	caches $1 sah bfs aabb whilewhile persistent sm 512 ${@:2}
	caches $1 med bfs aabb whilewhile persistent norm 512 ${@:2}
	caches $1 sah random aabb whilewhile persistent norm 512 ${@:2}
	caches $1 sah bfs aabb ifif persistent norm 512 ${@:2}
	caches $1 sah bfs aabb whilewhile persistent norm 2k ${@:2}
}


# matrices:
# matrixfigure xyzrgb_dragon.ply 60.011566 0.110699 200 10 0 0 0 10 0 0 0 10 30
# matrixfigure san-miguel.ply 25.5 1.75226 1.72598 -0.318385 -0.202748 0.926026 0.00768433 0.976276 0.216392 -0.94793 0.076012 -0.309274 70

# timeline:
# simple_run sponza.ply sah random aabb ifif persistent keep 512 -744.366211 118.459343 -32.743423 -0.656636 0.1381 -0.741456 -0.00946993 0.981506 0.191197 0.754148 0.132568 -0.643184 60.481781
# simple_run bun_zipper.ply sah random aabb ifif persistent keep 512 -0.02 0.10 0.19 1 0 0 0 1 0 0 0 1 70

# for kais dumps:
# simple_run sponza.ply sah random aabb ifif persistent keep 256 -744.366211 118.459343 -32.743423 -0.656636 0.1381 -0.741456 -0.00946993 0.981506 0.191197 0.754148 0.132568 -0.643184 60.481781
simple_run bun_zipper.ply sah random aabb ifif persistent keep 256 -0.02 0.10 0.19 1 0 0 0 1 0 0 0 1 70

# measured:
# cachesmatrixfigure san-miguel.ply 25.5 1.75226 1.72598 -0.318385 -0.202748 0.926026 0.00768433 0.976276 0.216392 -0.94793 0.076012 -0.309274 70
# cachesmatrixfigure xyzrgb_dragon.ply 60.011566 0.110699 200 10 0 0 0 10 0 0 0 10 30
