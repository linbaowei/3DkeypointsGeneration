#!/bin/sh

RUN_BUNDLER=./RunBundler_siftgpu.sh;
BUNDLE2PMVS=Bundle2PMVS;
PMVS2=pmvs2;
level_pmvs2=2;
SIFT=../uenoler/sift
trainingImagesDir=trainingImages;

echo "training images are in dir : $trainingImagesDir";
rm -rf bundle/ pmvs/ prepare/

# bundler
echo "$RUN_BUNDLER $trainingImagesDir";
$RUN_BUNDLER $trainingImagesDir

# bundle2pmvs: convert bundler output to PMVS input
echo "$BUNDLE2PMVS list.txt bundle/bundle.out";
$BUNDLE2PMVS list.txt bundle/bundle.out

echo "sh pmvs/prep_pmvs.sh";
sh pmvs/prep_pmvs.sh



# modify optionfiles for PMVS2
OPTIONS="pmvs/pmvs_options2.txt";

echo "level $level_pmvs2" > $OPTIONS;
echo "csize 2" >> $OPTIONS;
echo "threshold 0.7" >> $OPTIONS;
echo "wsize 7" >> $OPTIONS;
echo "minImageNum 3" >> $OPTIONS;
echo "CPU 4" >> $OPTIONS;
echo "setEdge 0" >> $OPTIONS;
echo "useBound 0" >> $OPTIONS;
echo "useVisData 1" >> $OPTIONS;
echo "sequence -1" >> $OPTIONS;
grep "^timages" pmvs/pmvs_options.txt >> $OPTIONS;
echo "oimages 0" >> $OPTIONS;


# run PMVS2
echo "$PMVS2 pmvs/ pmvs_options2.txt";
$PMVS2 pmvs/ pmvs_options2.txt


# put all results into a dir

OUTDIR=training3Dpoints;
mkdir $OUTDIR

mv bundle pmvs prepare constraints.txt list.txt list_keys.txt list_tmp.txt \
 matches.corresp.txt matches.init.txt matches.prune.txt matches.ransac.txt \
 nmatches.corresp.txt nmatches.prune.txt nmatches.ransac.txt options.txt pairwise_scores.txt sift.txt \
 $OUTDIR


mv $OUTDIR/pmvs/models/pmvs_options2.txt.ply $OUTDIR/PMVS2output_level$level_pmvs2.ply
mv $OUTDIR/pmvs/txt $OUTDIR/ProjectionMatrices
mv $OUTDIR/pmvs/visualize $OUTDIR/PMVS2inputImages




myfile=./training3Dpoints/pmvs/list.rd.txt
line_no=1
while read my_line
  do
    #echo $my_line
    newstr=${my_line#*/}
    #echo ${newstr%.*}
    cp  ./trainingImages/${newstr%.*}.key.gz $OUTDIR/PMVS2inputImages/$((line_no-1)).key.gz
    gzip -df $OUTDIR/PMVS2inputImages/$((line_no-1)).key.gz
    rm -f $OUTDIR/PMVS2inputImages/$((line_no-1)).key.gz
    line_no=$((line_no+1))
done < $myfile
