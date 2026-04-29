#!/usr/bin/env python3
"""
PGM 맵 파일 비교 도구
생성된 맵 이미지들을 시각적으로 비교
"""

import numpy as np
from pathlib import Path
from datetime import datetime
import argparse
import yaml
import json


class PGMAnalyzer:
    def __init__(self, data_dir):
        self.data_dir = Path(data_dir)
    
    def read_pgm(self, pgm_file):
        """PGM 파일 읽기"""
        try:
            with open(pgm_file, 'rb') as f:
                # PGM 헤더 읽기
                magic = f.readline().decode().strip()
                if magic not in ['P5', 'P2']:
                    print(f"⚠️  잘못된 PGM 파일: {magnetic}")
                    return None
                
                # 주석 건너뛰기
                while True:
                    line = f.readline().decode().strip()
                    if not line.startswith('#'):
                        break
                    width, height = map(int, line.split())
                
                # 이미지 정보 파싱
                info_line = f.readline().decode().strip()
                width, height = map(int, info_line.split())
                
                max_value = int(f.readline().decode().strip())
                
                # 이미지 데이터 읽기
                if magic == 'P5':
                    # 바이너리 형식
                    data = np.fromfile(f, dtype=np.uint8, count=width*height)
                else:
                    # 텍스트 형식
                    data = np.fromfile(f, dtype=np.uint8, sep=' ')
                
                image = data.reshape((height, width))
                
                return {
                    'image': image,
                    'width': width,
                    'height': height,
                    'max_value': max_value,
                    'format': magic
                }
        
        except Exception as e:
            print(f"❌ 오류: {e}")
            return None
    
    def read_yaml(self, yaml_file):
        """YAML 메타데이터 읽기"""
        try:
            with open(yaml_file, 'r') as f:
                return yaml.safe_load(f)
        except:
            return None
    
    def find_pgm_files(self):
        """모든 PGM 파일 찾기"""
        pgm_files = sorted(self.data_dir.glob('*.pgm'))
        print(f"\n📂 찾은 PGM 파일: {len(pgm_files)}")
        for i, f in enumerate(pgm_files, 1):
            print(f"   {i}. {f.name}")
        return pgm_files
    
    def analyze_pgm(self, pgm_file):
        """PGM 파일 분석"""
        print(f"\n🔍 분석 중: {pgm_file.name}")
        print("-" * 70)
        
        # PGM 파일 읽기
        pgm_data = self.read_pgm(pgm_file)
        if not pgm_data:
            return None
        
        image = pgm_data['image']
        
        # YAML 메타데이터 읽기
        yaml_file = pgm_file.parent / f"{pgm_file.stem}.yaml"
        yaml_data = None
        if yaml_file.exists():
            yaml_data = self.read_yaml(yaml_file)
        
        # 분석
        analysis = {
            'filename': pgm_file.name,
            'width': pgm_data['width'],
            'height': pgm_data['height'],
            'resolution': yaml_data.get('resolution', 0) if yaml_data else 0,
            'origin': yaml_data.get('origin', [0, 0, 0]) if yaml_data else [0, 0, 0],
            'occupied_threshold': yaml_data.get('occupied_threshold', 0) if yaml_data else 0,
            'free_threshold': yaml_data.get('free_threshold', 0) if yaml_data else 0,
            'negate': yaml_data.get('negate', False) if yaml_data else False,
            'image_stats': {
                'min': int(image.min()),
                'max': int(image.max()),
                'mean': float(image.mean()),
                'std': float(image.std()),
                'occupied_pixels': int(np.sum(image < 128)),  # 어두운 픽셀 = 점유
                'free_pixels': int(np.sum(image > 200)),      # 밝은 픽셀 = 자유
                'unknown_pixels': int(np.sum((image >= 128) & (image <= 200)))  # 회색 = 미지
            }
        }
        
        return analysis
    
    def analyze_all_pgms(self):
        """모든 PGM 파일 분석"""
        pgm_files = self.find_pgm_files()
        
        if not pgm_files:
            print("⚠️  PGM 파일이 없습니다")
            return
        
        print("\n" + "="*80)
        print("📊 PGM 맵 분석")
        print("="*80)
        
        all_analyses = []
        
        for pgm_file in pgm_files:
            analysis = self.analyze_pgm(pgm_file)
            
            if analysis:
                all_analyses.append(analysis)
                
                print("\n📊 맵 정보:")
                print(f"   파일명: {analysis['filename']}")
                print(f"   크기: {analysis['width']} x {analysis['height']} pixels")
                print(f"   해상도: {analysis['resolution']} m/pixel")
                print(f"   원점(origin): ({analysis['origin'][0]}, {analysis['origin'][1]})")
                
                stats = analysis['image_stats']
                print(f"\n🔍 이미지 통계:")
                print(f"   점유 픽셀: {stats['occupied_pixels']:,} ({100*stats['occupied_pixels']/len(pgm_data['image'].flatten()):.1f}%)" if 'pgm_data' in locals() else f"   점유 픽셀: {stats['occupied_pixels']:,}")
                print(f"   자유 픽셀: {stats['free_pixels']:,}")
                print(f"   미지 픽셀: {stats['unknown_pixels']:,}")
                print(f"   평균값: {stats['mean']:.1f}")
                print(f"   표준편차: {stats['std']:.1f}")
        
        # 비교 테이블
        if len(all_analyses) > 1:
            self.print_comparison_table(all_analyses)
        
        # 결과 저장
        self.save_analysis(all_analyses)
    
    def print_comparison_table(self, analyses):
        """비교 테이블 출력"""
        print("\n" + "="*80)
        print("📊 비교 테이블")
        print("="*80 + "\n")
        
        print(f"{'파일명':<25} | {'크기':>15} | {'해상도':>8} | {'점유%':>7} | {'자유%':>7}")
        print("-" * 80)
        
        for analysis in analyses:
            size = f"{analysis['width']}x{analysis['height']}"
            stats = analysis['image_stats']
            total_pixels = stats['occupied_pixels'] + stats['free_pixels'] + stats['unknown_pixels']
            occupied_pct = 100 * stats['occupied_pixels'] / total_pixels if total_pixels > 0 else 0
            free_pct = 100 * stats['free_pixels'] / total_pixels if total_pixels > 0 else 0
            
            print(f"{analysis['filename']:<25} | {size:>15} | {analysis['resolution']:>8.2f} | {occupied_pct:>6.1f}% | {free_pct:>6.1f}%")
        
        print("\n💡 해석:")
        print("   점유%: 작은 값 = 지도에 적은 장애물 (개방된 환경)")
        print("   자유%: 큰 값 = 더 안정적인 매핑 (충분한 오픈 스페이스 스캔)")
    
    def save_analysis(self, analyses):
        """분석 결과 저장"""
        output_file = self.data_dir / f"pgm_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        with open(output_file, 'w') as f:
            json.dump(analyses, f, indent=2)
        
        print(f"\n💾 분석 결과 저장: {output_file}")
        
        # CSV로도 저장
        try:
            import csv
            csv_file = self.data_dir / f"pgm_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            
            with open(csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                
                # 헤더
                if analyses:
                    headers = ['filename', 'width', 'height', 'resolution', 'occupied_pixels', 'free_pixels', 'unknown_pixels', 'mean_value', 'std_value']
                    writer.writerow(headers)
                    
                    # 데이터
                    for analysis in analyses:
                        stats = analysis['image_stats']
                        writer.writerow([
                            analysis['filename'],
                            analysis['width'],
                            analysis['height'],
                            f"{analysis['resolution']:.4f}",
                            stats['occupied_pixels'],
                            stats['free_pixels'],
                            stats['unknown_pixels'],
                            f"{stats['mean']:.2f}",
                            f"{stats['std']:.2f}"
                        ])
            
            print(f"💾 CSV 저장: {csv_file}")
        except ImportError:
            pass


def main():
    parser = argparse.ArgumentParser(description='PGM 맵 파일 분석')
    parser.add_argument('--data-dir', default='/home/symoon/SLAM_main', help='데이터 디렉토리')
    parser.add_argument('--file', help='특정 파일 분석')
    
    args = parser.parse_args()
    
    analyzer = PGMAnalyzer(args.data_dir)
    
    print("\n" + "="*80)
    print("🗺️  PGM Map Analysis Tool")
    print("="*80)
    
    if args.file:
        pgm_file = Path(args.data_dir) / args.file
        if pgm_file.exists():
            analysis = analyzer.analyze_pgm(pgm_file)
            if analysis:
                print(f"\n✅ 분석 완료")
        else:
            print(f"⚠️  파일을 찾을 수 없습니다: {pgm_file}")
    else:
        analyzer.analyze_all_pgms()


if __name__ == '__main__':
    main()
