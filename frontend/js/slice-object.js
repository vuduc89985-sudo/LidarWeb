(function(root){
  'use strict';
  const defaults=Object.freeze({clusterEps:0.20,clusterMinPoints:3,enableContour:true,enableDebugRender:true,
    // Point clouds from the current sensors are 2D scan planes.  This is deliberately
    // a small descriptive profile, not an inferred solid object model.
    zProfileBins:12,maxPointsPerSensor:2500});

  function pointInPolygon(vertices,x,y){
    if(!vertices||vertices.length<3)return false;let inside=false;
    for(let i=0,j=vertices.length-1;i<vertices.length;j=i++){
      const a=vertices[i],b=vertices[j];
      if(((a.y>y)!==(b.y>y))&&(x<(b.x-a.x)*(y-a.y)/(b.y-a.y)+a.x))inside=!inside;
    }
    return inside;
  }

  function dbscanIndices(points,eps,minPoints){
    const buckets=new Map(),labels=new Int32Array(points.length);labels.fill(-2);
    const key=(x,y)=>x+','+y,cell=p=>[Math.floor(p.localX/eps),Math.floor(p.localY/eps)];
    points.forEach((p,i)=>{const c=cell(p),k=key(c[0],c[1]);if(!buckets.has(k))buckets.set(k,[]);buckets.get(k).push(i);});
    const neighbors=i=>{const p=points[i],c=cell(p),out=[],e2=eps*eps;
      for(let dx=-1;dx<=1;dx++)for(let dy=-1;dy<=1;dy++)for(const j of buckets.get(key(c[0]+dx,c[1]+dy))||[]){
        const q=points[j],x=p.localX-q.localX,y=p.localY-q.localY;if(x*x+y*y<=e2)out.push(j);
      }return out;};
    let id=0;
    for(let i=0;i<points.length;i++){
      if(labels[i]!==-2)continue;const first=neighbors(i);if(first.length<minPoints){labels[i]=-1;continue;}
      labels[i]=id;const queue=[];
      for(const j of first)if(labels[j]===-2){labels[j]=id;if(j!==i)queue.push(j);}else if(labels[j]===-1)labels[j]=id;
      for(let q=0;q<queue.length;q++){const near=neighbors(queue[q]);if(near.length>=minPoints)for(const j of near){
        if(labels[j]===-2){labels[j]=id;queue.push(j);}else if(labels[j]===-1)labels[j]=id;
      }}id++;
    }
    return Array.from({length:id},(_,cluster)=>{const out=[];for(let i=0;i<labels.length;i++)if(labels[i]===cluster)out.push(i);return out;});
  }

  function convexHullIndices(points,indices){
    if(indices.length<=2)return indices.slice();
    const sorted=indices.slice().sort((a,b)=>points[a].localX-points[b].localX||points[a].localY-points[b].localY);
    const cross=(a,b,c)=>(points[b].localX-points[a].localX)*(points[c].localY-points[a].localY)-(points[b].localY-points[a].localY)*(points[c].localX-points[a].localX);
    const lower=[];for(const i of sorted){while(lower.length>=2&&cross(lower.at(-2),lower.at(-1),i)<=0)lower.pop();lower.push(i);}
    const upper=[];for(let k=sorted.length-1;k>=0;k--){const i=sorted[k];while(upper.length>=2&&cross(upper.at(-2),upper.at(-1),i)<=0)upper.pop();upper.push(i);}
    lower.pop();upper.pop();return lower.concat(upper);
  }

  function buildZProfile(slicePoints,zMin,zMax,bins){
    if(!Number.isFinite(zMin)||!Number.isFinite(zMax)||!slicePoints.length)return [];
    const count=Math.max(1,Math.floor(bins||defaults.zProfileBins)),span=zMax-zMin;
    const out=Array.from({length:count},(_,index)=>({index,zMin:zMin+(span*index/count),zMax:zMin+(span*(index+1)/count),pointCount:0,state:'unknown'}));
    for(const p of slicePoints){
      const index=span>1e-6?Math.min(count-1,Math.max(0,Math.floor((p.worldZ-zMin)/span*count))):0;
      out[index].pointCount++;
    }
    for(const bin of out)if(bin.pointCount>0)bin.state='occupied';
    // A scan has no ray-complete 3D coverage, so zero-count bins remain unknown,
    // never "empty".  This prevents occluded/unseen height from becoming negative evidence.
    return out;
  }

  function buildSliceObject(points,indices,sensorId,sensorName,roiId,timestamp,frameId,clusterIndex,enableContour,config={}){
    let lminX=Infinity,lminY=Infinity,lmaxX=-Infinity,lmaxY=-Infinity,wminX=Infinity,wminY=Infinity,wminZ=Infinity,wmaxX=-Infinity,wmaxY=-Infinity,wmaxZ=-Infinity;
    let lsx=0,lsy=0,wsx=0,wsy=0,wsz=0;
    const slicePoints=indices.map(i=>points[i]);
    for(const p of slicePoints){
      lminX=Math.min(lminX,p.localX);lmaxX=Math.max(lmaxX,p.localX);lminY=Math.min(lminY,p.localY);lmaxY=Math.max(lmaxY,p.localY);lsx+=p.localX;lsy+=p.localY;
      wminX=Math.min(wminX,p.worldX);wmaxX=Math.max(wmaxX,p.worldX);wminY=Math.min(wminY,p.worldY);wmaxY=Math.max(wmaxY,p.worldY);wminZ=Math.min(wminZ,p.worldZ);wmaxZ=Math.max(wmaxZ,p.worldZ);wsx+=p.worldX;wsy+=p.worldY;wsz+=p.worldZ;
    }
    const hull=enableContour?convexHullIndices(points,indices):[];
    const sensorPositionWorld={x:config.sensorPositionWorld?.x??0,y:config.sensorPositionWorld?.y??0,z:config.sensorPositionWorld?.z??0};
    const center={x:wsx/slicePoints.length,y:wsy/slicePoints.length,z:wsz/slicePoints.length};
    const dx=center.x-sensorPositionWorld.x,dy=center.y-sensorPositionWorld.y,dz=center.z-sensorPositionWorld.z,viewLength=Math.hypot(dx,dy,dz);
    const ranges=slicePoints.map(p=>Math.hypot(p.worldX-sensorPositionWorld.x,p.worldY-sensorPositionWorld.y,p.worldZ-sensorPositionWorld.z));
    const zProfile=buildZProfile(slicePoints,wminZ,wmaxZ,config.zProfileBins);
    return {sliceId:'L'+(sensorId+1)+'-R'+roiId+'-F'+frameId+'-C'+clusterIndex,sensorId,sensorName,roiId,timestamp,frameId,pointCount:slicePoints.length,
      local:{center:{x:lsx/slicePoints.length,y:lsy/slicePoints.length},bbox:{minX:lminX,maxX:lmaxX,minY:lminY,maxY:lmaxY},width:lmaxX-lminX,depth:lmaxY-lminY,
        contour:hull.map(i=>({x:points[i].localX,y:points[i].localY}))},
      world:{center,bbox:{minX:wminX,maxX:wmaxX,minY:wminY,maxY:wmaxY,minZ:wminZ,maxZ:wmaxZ},
        contour:hull.map(i=>({x:points[i].worldX,y:points[i].worldY,z:points[i].worldZ}))},
      geometry3D:{zMin:wminZ,zMax:wmaxZ,observedHeight:wmaxZ-wminZ,zCenter:center.z},zProfile,
      visibility:{state:'observed',hasOcclusionEvidence:false,visibleRangeMin:ranges.length?Math.min(...ranges):null,visibleRangeMax:ranges.length?Math.max(...ranges):null},
      view:{sensorPositionWorld,viewDirectionWorld:viewLength>1e-6?{x:dx/viewLength,y:dy/viewLength,z:dz/viewLength}:null},points:slicePoints};
  }

  function processSensorSlices(frame,localRois,sensorName,config={}){
    const c={...defaults,...config},sensorId=frame.sourceId,timestamp=frame.timestamp,frameId=frame.frameId,allPoints=[];
    const inputPointCount=Math.floor(frame.pts.length/3),sampleStride=Math.max(1,Math.ceil(inputPointCount/Math.max(1,c.maxPointsPerSensor)));
    // Analysis intentionally uses an even, deterministic sample only. Rendering and
    // WebSocket frames retain every point; this prevents DBSCAN from blocking the UI.
    for(let pointIndex=0;pointIndex<inputPointCount;pointIndex+=sampleStride){const i=pointIndex*3;allPoints.push({localX:frame.pts[i],localY:frame.pts[i+1],worldX:frame.xyz[i],worldY:frame.xyz[i+1],worldZ:frame.xyz[i+2],intensity:frame.pts[i+2],sensorId,timestamp});}
    const slices=[];let roiPointCount=0,clusterCount=0;
    for(const roi of localRois){
      const filtered=allPoints.filter(p=>pointInPolygon(roi.vertices,p.localX,p.localY));roiPointCount+=filtered.length;
      const clusters=dbscanIndices(filtered,c.clusterEps,c.clusterMinPoints);clusterCount+=clusters.length;
      const sensorPositionWorld={x:frame.ox??0,y:frame.oy??0,z:frame.oz??0};
      clusters.forEach((indices,i)=>slices.push(buildSliceObject(filtered,indices,sensorId,sensorName,roi.roiId,timestamp,frameId,i,c.enableContour,{...c,sensorPositionWorld})));
    }
    return {slices,stats:{roiPoints:roiPointCount,clusters:clusterCount,slices:slices.length,inputPoints:inputPointCount,sampledPoints:allPoints.length,sampleStride}};
  }

  function processSliceFrame(frames,localRoisBySensor,sensorNames=[],config={}){
    const bySensor={},stats={};let timestamp=null;
    for(const frame of frames.filter(Boolean)){
      const id=frame.sourceId,result=processSensorSlices(frame,localRoisBySensor[id]||[],sensorNames[id]||('L'+(id+1)),config);
      bySensor[id]=result.slices;stats[id]=result.stats;timestamp=timestamp===null?frame.timestamp:Math.max(timestamp,frame.timestamp);
    }
    return {timestamp,bySensor,stats};
  }
  root.SliceClustering={defaults,pointInPolygon,dbscanIndices,convexHullIndices,buildSliceObject,processSensorSlices,processSliceFrame};
  if(typeof module!=='undefined')module.exports=root.SliceClustering;
})(globalThis);
