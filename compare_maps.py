#!/usr/bin/env python3
"""
Cartographer Ground Truth 기반 맵 비교 도구
여러 pgm/yaml 파일들의 정확도를 비교
"""

import subprocess
import json
import os
from pathlib import Path
from datetime import datetime
import argparse


class MapComparator:
    def __init__(self, data_dir, build_dir):
        self.data_dir = Path(data_dir)
        self.build_dir = Path(build_dir)
        
        # Cartographer 도구 경로
        self.compute_relations_metrics = self.build_dir / 'cartographer' / 'cartographer_compute_relations_metrics'
        self.autogenerate_ground_truth = self.build_dir / 'cartographer' / 'cartographer_autogenerate_ground_truth'
        
        # 도구 확인
        if not self.compute_relations_metrics.exists():
            print(f"⚠️  {self.compute_relations_metrics} not found!")
        if not self.autogenerate_ground_truth.exists():
            print(f"⚠️  {self.autogenerate_ground_truth} not found!")
    
    def find_pbstream_files(self):
        """모든 pbstream 파일 찾기"""
        pbstream_files = list(self.data_dir.glob('*.pbstream'))
        print(f"\n📂 찾은 pbstream 파일: {len(pbstream_files)}")
        for f in sorted(pbstream_files):
            print(f"   - {f.name}")
        return sorted(pbstream_files)
    
    def find_yaml_files(self):
        """모든 yaml 파일 찾기"""
        yaml_files = list(self.data_dir.glob('*.yaml'))
        print(f"\n📋 찾은 YAML 파일: {len(yaml_files)}")
        for f in sorted(yaml_files):
            print(f"   - {f.name}")
        return sorted(yaml_files)
    
    def compare_maps(self, pbstream_file1, pbstream_file2):
        """두 맵 비교"""
        print(f"\n🔍 비교: {pbstream_file1.name} <-> {pbstream_file2.name}")
        print("-" * 70)
        
        try:
            # ground truth 파일 경로
            gt_file = pbstream_file1.parent / f"relations_{pbstream_file1.stem}_vs_{pbstream_file2.stem}.txt"
            
            # compute_relations_metrics 실행
            cmd = [
                str(self.compute_relations_metrics),
                f"--pose_graph_filename={pbstream_file1}",
                f"--relations_filename={gt_file}",
                f"--read_text_file_with_unix_timestamps",
                f"--write_relation_metrics"
            ]
            
            print(f"📊 실행 중...")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                print("✅ 분석 완료")
                if result.stdout:
                    print("\n📈 결과:")
                    print(result.stdout)
                return True
            else:
                print(f"⚠️  오류: {result.stderr}")
                return False
        
        except subprocess.TimeoutExpired:
            print("⏱️  타임아웃 (30초)")
            return False
        except Exception as e:
            print(f"❌ 오류: {e}")
            return False
    
    def analyze_map_quality(self, pbstream_file):
        """맵 품질 분석"""
        print(f"\n🔍 맵 품질 분석: {pbstream_file.name}")
        print("-" * 70)
        
        try:
            # 더미 ground truth 파일로 분석
            gt_file = pbstream_file.parent / f"dummy_relations_{pbstream_file.stem}.txt"
            
            # cartographer_pbstream을 사용해서 정보 추출
            cmd_info = [
                str(self.build_dir / 'cartographer' / 'cartographer_pbstream'),
                f"--mode=info",
                f"--pbstream_filename={pbstream_file}"
            ]
            
            print(f"📊 실행 중...")
            result = subprocess.run(cmd_info, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                print("✅ 분석 완료")
                if result.stdout:
                    print("\n📈 결과:")
                    print(result.stdout)
                return True
            else:
                # 대체 명령
                print(f"ℹ️  pbstream 정보 출력...")
                return False
        
        except Exception as e:
            print(f"❌ 오류: {e}")
            return False
    
    def list_map_pairs(self):
        """비교 가능한 맵 쌍 나열"""
        pbstream_files = self.find_pbstream_files()
        
        if len(pbstream_files) < 2:
            print("\n⚠️  비교할 맵이 최소 2개 필요합니다")
            return []
        
        print("\n🔗 비교 가능한 맵 쌍:")
        pairs = []
        for i in range(len(pbstream_files)):
            for j in range(i + 1, len(pbstream_files)):
                pairs.append((pbstream_files[i], pbstream_files[j]))
                print(f"   {i+1}. {pbstream_files[i].name} vs {pbstream_files[j].name}")
        
        return pairs


def main():
    parser = argparse.ArgumentParser(description='Cartographer 맵 비교 도구')
    parser.add_argument('--data-dir', default='/home/symoon/SLAM_main', help='데이터 디렉토리')
    parser.add_argument('--build-dir', default='/home/symoon/SLAM_main/build', help='빌드 디렉토리')
    parser.add_argument('--compare', help='두 파일 비교 (예: 0119_1.pbstream,0120.pbstream)')
    parser.add_argument('--analyze', help='단일 맵 분석')
    parser.add_argument('--list', action='store_true', help='사용 가능한 맵 나열')
    
    args = parser.parse_args()
    
    comparator = MapComparator(args.data_dir, args.build_dir)
    
    print("\n" + "="*70)
    print("🗺️  Cartographer Map Comparison Tool")
    print("="*70)
    
    # pbstream 파일 찾기
    pbstream_files = comparator.find_pbstream_files()
    yaml_files = comparator.find_yaml_files()
    
    if args.list:
        comparator.list_map_pairs()
        return
    
    if args.compare:
        files = args.compare.split(',')
        file1 = comparator.data_dir / files[0]
        file2 = comparator.data_dir / files[1]
        
        if file1.exists() and file2.exists():
            comparator.compare_maps(file1, file2)
        else:
            print("⚠️  파일을 찾을 수 없습니다")
    
    elif args.analyze:
        analyze_file = comparator.data_dir / args.analyze
        if analyze_file.exists():
            comparator.analyze_map_quality(analyze_file)
        else:
            print("⚠️  파일을 찾을 수 없습니다")
    
    else:
        # 기본: 모든 쌍 비교
        pairs = comparator.list_map_pairs()
        
        if pairs:
            print("\n💡 사용법:")
            print(f"   python3 compare_maps.py --compare 0119_1.pbstream,0120.pbstream")
            print(f"   python3 compare_maps.py --analyze 0119_1.pbstream")
            print(f"   python3 compare_maps.py --list")


if __name__ == '__main__':
    main()
