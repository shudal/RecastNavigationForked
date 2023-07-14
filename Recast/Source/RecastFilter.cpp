//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "Recast.h"
#include "RecastAssert.h"

#include <stdlib.h>

/*
迭代每一列，从下往上遍历span，对于同列任意两个相邻的span1（下）和span2（上），当span1可走，并且span2不可走的时候，计算这两个span的上表面高度差Diff，如果Diff小于配置参数“walkableClimb”，则将span2设置为“可走”
*/
// 之前体素化时，也用过这个walkableClimb，不过那时的处理 并不会处理到这个函数处理的情况。因为那时把新的插进去了，就不管比这个新的要高的了。
void rcFilterLowHangingWalkableObstacles(rcContext* context, const int walkableClimb, rcHeightfield& heightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_LOW_OBSTACLES);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			rcSpan* previousSpan = NULL;
			bool previousWasWalkable = false;
			unsigned char previousArea = RC_NULL_AREA;

			for (rcSpan* span = heightfield.spans[x + z * xSize]; span != NULL; previousSpan = span, span = span->next)
			{
				const bool walkable = span->area != RC_NULL_AREA;
				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.
				if (!walkable && previousWasWalkable)
				{
					if (rcAbs((int)span->smax - (int)previousSpan->smax) <= walkableClimb)
					{
						span->area = previousArea;
					}
				}
				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				previousWasWalkable = walkable;
				previousArea = span->area;
			}
		}
	}
}

// 当一个span 傍边有span比自己低的多（walkableClimb）时，此span不可走
// 当一个span 有两个邻居span的 上表面高度差 > walkableClimb时，此span不可走
// 这两种情况下，当前span起了隔断作用

// 其他情况
//		1 当前span上面的区间高度 < walkableHeight，此函数不会将当前span设为不可走。后续调用的 rcFilterWalkableLowHeightSpans 可以处理这种情况。
//		2 邻居区间 都不是有效邻居区间，此函数也不会将当前span设为不可走。后续创建compact height field时，会将自己和这样的邻居设为不连通。
//		 
void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb,
                        rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff; // TODO (graham): Move this to a more visible constant and update usages.
	
	// Mark border spans.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next)
			{
				// Skip non walkable spans.
				if (span->area == RC_NULL_AREA)
				{
					continue;
				}

				// 这个bottom和top，表示是这个空余的区间的bottom和top
				const int bot = (int)(span->smax);
				const int top = span->next ? (int)(span->next->smin) : MAX_HEIGHT;

				// Find neighbours minimum height.
				// 邻居区间下表面 - 当前区间下表面。<0时说明当前span（悬崖）傍边 有 比自己矮的多的span，此时自己为 峭壁leadge span
				int minNeighborHeight = MAX_HEIGHT;

				
				// Min and max height of accessible neighbours.
				// 邻居区间 底的最小值
				int accessibleNeighborMinHeight = span->smax;
				// 邻居区间 底的最大值
				int accessibleNeighborMaxHeight = span->smax;

				for (int direction = 0; direction < 4; ++direction)
				{
					int dx = x + rcGetDirOffsetX(direction);
					int dy = z + rcGetDirOffsetY(direction);
					// Skip neighbours which are out of bounds.
					if (dx < 0 || dy < 0 || dx >= xSize || dy >= zSize)
					{
						// 邻居超界了，那邻居的区间一定是 有效邻居区间，直接设置 minNeighborHeight 以便后续判断 当前span是否为leage span
						
						// 邻居区间下表面 - 当前区间下表面；邻居区间下表面直接设为  -walkableClimb
						minNeighborHeight = rcMin(minNeighborHeight, -walkableClimb - bot);
						continue;
					}

					// From minus infinity to the first span.
					const rcSpan* neighborSpan = heightfield.spans[dx + dy * xSize];
					int neighborBot = -walkableClimb;
					int neighborTop = neighborSpan ? (int)neighborSpan->smin : MAX_HEIGHT;
					
					// 相当于是把当前span上面的 这个区间 和邻居column的【地面 到 第一个span的底部】这个的区间 比较。判断碰头不碰头
					// Skip neighbour if the gap between the spans is too small.
					if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
					{
						// 不碰头的话，
						
						// 邻居区间下表面 - 当前区间下表面
						minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);
					}

					// Rest of the spans.
					for (neighborSpan = heightfield.spans[dx + dy * xSize]; neighborSpan; neighborSpan = neighborSpan->next)
					{
						neighborBot = (int)neighborSpan->smax;
						neighborTop = neighborSpan->next ? (int)neighborSpan->next->smin : MAX_HEIGHT;
						
						// Skip neighbour if the gap between the spans is too small.
						if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
						{
							// 因为 walkableHeight > 0，所有上面的if相当于也限制了top>neighborBot,netighborTop>bot
							// 这个限制 保证了 从当前span走到邻居span时，上下坡不碰头

							minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);

							// 当minNeighborHeight < -walkableClimb时，后面代码可以判断出当前span不可走；在这种情况下rcAbs(neighborBot - bot) <= walkableClimb 必然不符合。
							// Find min/max accessible neighbour height. 
							if (rcAbs(neighborBot - bot) <= walkableClimb)
							{
								if (neighborBot < accessibleNeighborMinHeight) accessibleNeighborMinHeight = neighborBot;
								if (neighborBot > accessibleNeighborMaxHeight) accessibleNeighborMaxHeight = neighborBot;
							}

						}
					}
				}

				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
				if (minNeighborHeight < -walkableClimb)
				{
					// 傍边是悬崖
					// 峭壁区间，不可走
					span->area = RC_NULL_AREA;
				}
				// 邻居的 上表面 差距过大，把当前span设为不可走，来隔断这两个。
				// If the difference between all neighbours is too large,
				// we are at steep slope, mark the span as ledge.
				else if ((accessibleNeighborMaxHeight - accessibleNeighborMinHeight) > walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

void rcFilterWalkableLowHeightSpans(rcContext* context, const int walkableHeight, rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_WALKABLE);
	
	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff;
	
	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z*xSize]; span; span = span->next)
			{
				const int bot = (int)(span->smax);
				const int top = span->next ? (int)(span->next->smin) : MAX_HEIGHT;
				if ((top - bot) <= walkableHeight)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}
