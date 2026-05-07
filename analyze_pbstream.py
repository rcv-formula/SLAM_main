#!/usr/bin/env python3
"""
Cartographer pbstream 분석 도구
저장된 SLAM 결과를 분석하고 비교
"""

import subprocess
import json
import os
from pathlib import Path
from datetime import datetime
import argparse
import re


class PbstreamAnalyzer:
    def __init__(self, data_dir, cartographer_pbstream_path):
        self.data_dir = Path(data_dir)
        self.pbstream_tool = Path(cartographer_pbstream_path)
        
        if not self.pbstream_tool.exists():
            print(f"⚠️  {self.pbstream_tool} not found!")
    
    def find_pbstream_files(self):
        """모든 pbstream 파일 찾기"""
        pbstream_files = sorted(self.data_dir.glob('*.pbstream'))
        print(f"\n📂 찾은 pbstream 파일: {len(pbstream_files)}")
        for i, f in enumerate(pbstream_files, 1):
            print(f"   {i}. {f.name}")
        return pbstream_files
    
    def get_pbstream_info(self, pbstream_file):
        """pbstream 파일의 정보 추출"""
        try:
            cmd = [
                str(self.pbstream_tool),
                'info',
                str(pbstream_file)
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                return result.stdout
            else:
                print(f"⚠️  오류: {result.stderr}")
                return None
        
        except subprocess.TimeoutExpired:
            print(f"⏱️  타임아웃 (30초)")
            return None
        except Exception as e:
            print(f"❌ 오류: {e}")
            return None
    
    def parse_pbstream_info(self, info_text):
        """pbstream info 결과 파싱"""
        data = {
            'raw': info_text,
            'submaps': 0,
            'trajectory_nodes': 0,
            'constraints': 0,
            'poses': []
        }
        
        if not info_text:
            return data
        
        # 정보 파싱
        lines = info_text.split('\n')
        for line in lines:
            if 'submap' in line.lower():
                try:
                    numbers = re.findall(r'\d+', line)
                    if numbers:
                        data['submaps'] = int(numbers[0])
                except:
                    pass
            
            if 'trajectory node' in line.lower() or 'node' in line.lower():
                try:
                    numbers = re.findall(r'\d+', line)
                    if numbers:
                        data['trajectory_nodes'] = int(numbers[0])
                except:
                    pass
            
            if 'constraint' in line.lower():
                try:
                    numbers = re.findall(r'\d+', line)
                    if numbers:
                        data['constraints'] = int(numbers[0])
                except:
                    pass
        
        return data
    
    def analyze_all_pbstreams(self):
        """모든 pbstream 파일 분석"""
        pbstream_files = self.find_pbstream_files()
        
        if not pbstream_files:
            print("⚠️  pbstream 파일이 없습니다")
            return
        
        print("\n" + "="*80)
        print("📊 SLAM 결과 분석")
        print("="*80)
        
        all_data = {}
        
        for pbstream_file in pbstream_files:
            print(f"\n🔍 분석 중: {pbstream_file.name}")
            print("-" * 80)
            
            info_text = self.get_pbstream_info(pbstream_file)
            
            if info_text:
                parsed_data = self.parse_pbstream_info(info_text)
                all_data[pbstream_file.name] = parsed_data
                
                print("\n📈 결과:")
                print(info_text)
                
                print("\n📊 요약:")
                print(f"   서브맵: {parsed_data['submaps']}")
                print(f"   트래젝토리 노드: {parsed_data['trajectory_nodes']}")
                print(f"   제약 조건: {parsed_data['constraints']}")
        
        # 비교 테이블 출력
        if len(all_data) > 1:
            self.print_comparison_table(all_data)
        
        # JSON으로 저장
        self.save_analysis(all_data)
    
    def print_comparison_table(self, all_data):
        """분석 결과 비교 테이블"""
        print("\n" + "="*80)
        print("📊 비교 테이블")
        print("="*80 + "\n")
        
        print(f"{'파일명':<30} | {'서브맵':>10} | {'노드':>10} | {'제약':>10}")
        print("-" * 70)
        
        for filename, data in sorted(all_data.items()):
            print(f"{filename:<30} | {data['submaps']:>10} | {data['trajectory_nodes']:>10} | {data['constraints']:>10}")
        
        # 통계
        submaps = [data['submaps'] for data in all_data.values() if data['submaps'] > 0]
        nodes = [data['trajectory_nodes'] for data in all_data.values() if data['trajectory_nodes'] > 0]
        constraints = [data['constraints'] for data in all_data.values() if data['constraints'] > 0]
        
        print("-" * 70)
        if submaps:
            print(f"{'평균':<30} | {sum(submaps)/len(submaps):>10.1f} | {sum(nodes)/len(nodes):>10.1f} | {sum(constraints)/len(constraints):>10.1f}")
        
        print("\n📈 해석:")
        if submaps:
            print(f"   서브맵 범위: {min(submaps)} ~ {max(submaps)}")
            print(f"   노드 범위: {min(nodes)} ~ {max(nodes)}")
            print(f"   제약 범위: {min(constraints)} ~ {max(constraints)}")
    
    def save_analysis(self, all_data):
        """분석 결과 저장"""
        output_file = self.data_dir / f"pbstream_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        # JSON 호환 형식으로 변환
        json_data = {}
        for filename, data in all_data.items():
            json_data[filename] = {
                'submaps': data['submaps'],
                'trajectory_nodes': data['trajectory_nodes'],
                'constraints': data['constraints']
            }
        
        with open(output_file, 'w') as f:
            json.dump(json_data, f, indent=2)
        
        print(f"\n💾 분석 결과 저장: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Cartographer pbstream 분석')
    parser.add_argument('--data-dir', default='/home/symoon/SLAM_main', help='데이터 디렉토리')
    parser.add_argument('--pbstream-tool', 
                       default='/home/symoon/SLAM_main/build/cartographer/cartographer_pbstream',
                       help='cartographer_pbstream 도구 경로')
    parser.add_argument('--file', help='특정 파일 분석')
    
    args = parser.parse_args()
    
    analyzer = PbstreamAnalyzer(args.data_dir, args.pbstream_tool)
    
    print("\n" + "="*80)
    print("🗺️  Cartographer pbstream Analysis Tool")
    print("="*80)
    
    if args.file:
        pbstream_file = Path(args.data_dir) / args.file
        if pbstream_file.exists():
            print(f"\n🔍 분석 중: {args.file}")
            info = analyzer.get_pbstream_info(pbstream_file)
            if info:
                print(info)
        else:
            print(f"⚠️  파일을 찾을 수 없습니다: {pbstream_file}")
    else:
        analyzer.analyze_all_pbstreams()


if __name__ == '__main__':
    main()
